#!/usr/bin/env python3
"""
grid_planner.py  ---  Lap ke hoach duong di Manhattan tren luoi marker 2D
==========================================================================
Cho marker dich -> sinh chuoi buoc (goto + turn) de xe di tu vi tri hien tai
toi do, chi di tien/lui/trai/phai (khong cheo), re 90 do o cho doi truc.

Luoi (khop markers.yaml):
  row = id % grid_rows      x = row * spacing      (+x = huong dau xe luc home)
  col = id // grid_rows     y = -col * spacing     (cot tang sang phai = -y)

Huong di <-> yaw the gioi (da do thuc te, KHOP code ben duoi):
  +x (row tang)  -> yaw   0
  -x (row giam)  -> yaw 180
  -y (col tang, sang PHAI) -> yaw -90   (quay phai = yaw giam)
  +y (col giam, sang TRAI) -> yaw +90   (quay trai = yaw tang)

Buoc sinh ra:
  {"type":"turn", "heading_deg": <goc tuyet doi>}
  {"type":"goto", "stop_marker": <id>, "stop_dist": <m>, "heading_deg": <huong di>}
     -> di theo huong heading_deg cho toi khi marker stop_marker cach <= stop_dist
"""

import math


def wrap_deg(a):
    """Goi goc (do) ve [-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def id_to_grid(mid, rows):
    """id -> (row, col)."""
    return mid % rows, mid // rows


def grid_to_id(row, col, rows):
    """(row, col) -> id."""
    return col * rows + row


class GridPlanner:
    def __init__(self, rows, cols, spacing, stop_dist=0.50):
        self.rows = rows
        self.cols = cols
        self.spacing = spacing
        self.stop_dist = stop_dist

    # Huong di (dx, dy theo o luoi) -> yaw the gioi (do)
    # Quan he dau da do chac chan (IMU): quay TRAI = yaw TANG (+), quay PHAI = yaw GIAM (-).
    #   Marker 8 o ben PHAI xe (col tang = -y) -> quay phai -> yaw AM (-90).
    def dir_to_heading(self, d_row, d_col):
        if d_row > 0:  return 0      # +x (di toi truoc)
        if d_row < 0:  return 180    # -x (di lui/quay dau)
        if d_col > 0:  return -90    # col tang = sang PHAI = quay phai = yaw -90
        if d_col < 0:  return 90     # col giam = sang TRAI = quay trai = yaw +90
        return None

    def neighbor_back(self, marker_id, heading_deg):
        """Marker xe LUI VE khi dang o marker_id, vua di vao theo heading_deg,
        roi lui nguoc lai 1 o. Tra ve id marker lien ke (hoac chinh no neu da
        o bien luoi, khong lui qua duoc)."""
        row, col = id_to_grid(marker_id, self.rows)
        h = round(wrap_deg(heading_deg))
        # Lui NGUOC huong di vao 1 o:
        if h == 0:        row -= 1     # di vao theo +x -> lui ve -x
        elif abs(h) == 180: row += 1   # di vao theo -x -> lui ve +x
        elif h == -90:    col -= 1     # di vao theo -y (col tang) -> lui ve col giam
        elif h == 90:     col += 1     # di vao theo +y (col giam) -> lui ve col tang
        # Gioi han trong luoi
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return marker_id           # da o bien, coi nhu khong doi
        return grid_to_id(row, col, self.rows)

    def plan(self, start_id, goal_id, start_heading_deg=0.0, skip_approach=False):
        """
        Sinh danh sach buoc di tu start_id -> goal_id (Manhattan, it re nhat).
        start_heading_deg: huong xe DANG quay.
        skip_approach: True -> KHONG chen pha tien toi giao diem xuat phat (dung
            khi xe vua LUI ra khoi marker, neu tien lai se di nguoc).
        Tra ve list cac buoc {type, ...}. Rong neu start == goal.
        """
        sr, sc = id_to_grid(start_id, self.rows)
        gr, gc = id_to_grid(goal_id, self.rows)

        steps = []
        if (sr, sc) == (gr, gc):
            return steps

        # Hai chang: doc theo row (truc x) va doc theo col (truc y).
        # Toi uu it re: neu da cung row hoac cung col -> 1 chang (0 re).
        # Neu khac ca hai -> 2 chang (1 re). Chon di truc co nhieu o hon
        # truoc cho muot (it anh huong, nhung deu dung); o day chon X truoc.

        legs = []  # moi leg: (d_row, d_col, marker_dich_cuoi_leg)

        # Leg 1: di doc truc X (thay doi row) toi row dich, giu nguyen col xuat phat
        if gr != sr:
            d_row = 1 if gr > sr else -1
            end_id = grid_to_id(gr, sc, self.rows)
            legs.append((d_row, 0, end_id))

        # Leg 2: di doc truc Y (thay doi col) toi col dich
        if gc != sc:
            d_col = 1 if gc > sc else -1
            end_id = grid_to_id(gr, gc, self.rows)
            legs.append((0, d_col, end_id))

        # Neu chang dau co huong KHAC voi huong xe dang quay -> xe phai TIEN
        # toi dung giao diem xuat phat (marker start_id) TRUOC khi re, neu khong
        # se re tai cho dung (lui ~0.5m khoi marker) -> lech luoi.
        if legs and not skip_approach:
            first_heading = self.dir_to_heading(legs[0][0], legs[0][1])
            if first_heading is not None and \
               abs(wrap_deg(first_heading - start_heading_deg)) > 5.0:
                # Tien toi giao diem marker xuat phat theo huong hien tai roi moi re.
                steps.append({"type": "goto",
                              "stop_marker": int(start_id),
                              "stop_dist": float(self.stop_dist),
                              "heading_deg": float(start_heading_deg),
                              "reach": "to_node"})

        # Chuyen legs -> buoc turn + goto.
        # Xe nang lung CHUI XUONG GAM pallet -> tam xe phai toi DUNG vi tri marker
        # (marker dat giua pallet). Moi leg (ke ca leg cuoi/dich) deu dung kieu
        # "to_node": tien toi khi tam xe trung toa do marker.
        #   - Leg giua (sap re): tien toi giao diem roi re.
        #   - Leg cuoi (dich): tien toi giua pallet de chui vao doi len.
        # Doan cuoi marker chui xuong gam -> di mu ~0.5m bang pose (wheel odom+IMU);
        # moc dinh vi cuoi (luc con thay marker o 0.5m) cho do chinh xac.
        for d_row, d_col, end_id in legs:
            heading = self.dir_to_heading(d_row, d_col)
            steps.append({"type": "turn", "heading_deg": float(heading)})
            steps.append({"type": "goto",
                          "stop_marker": int(end_id),
                          "stop_dist": float(self.stop_dist),
                          "heading_deg": float(heading),
                          "reach": "to_node"})
        return steps

    # =================================================================
    # Sinh chuoi buoc cho MOT chang start_id -> goal_id, KEM dong tac nang/ha.
    # lift_h: chieu cao cylinder (None = khong nang-ha).
    # back_after: sau nang-ha thi LUI RA khoi gam pallet (dung sau khi HA).
    # =================================================================
    def plan_leg(self, start_id, goal_id, start_heading_deg=0.0,
                 lift_h=None, back_after=False, action=None, skip_approach=False):
        steps = self.plan(start_id, goal_id, start_heading_deg, skip_approach=skip_approach)
        if steps:
            last = steps[-1]
            if lift_h is not None:
                last['lift_h'] = float(lift_h)
            if action:
                last['action'] = action
            last['back_after'] = bool(back_after)
        return steps

    def heading_after(self, start_id, goal_id, start_heading_deg=0.0):
        """Huong xe quay mat sau khi di xong chang (de noi chang ke)."""
        steps = self.plan(start_id, goal_id, start_heading_deg)
        for s in reversed(steps):
            if 'heading_deg' in s:
                return s['heading_deg']
        return start_heading_deg


# Test nhanh khi chay truc tiep
if __name__ == '__main__':
    p = GridPlanner(rows=4, cols=5, spacing=0.4)
    for goal in [8, 3, 19, 5]:
        print(f"\n0 -> {goal}:")
        for s in p.plan(0, goal):
            print("  ", s)