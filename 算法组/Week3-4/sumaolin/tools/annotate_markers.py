"""Fast full-frame marker annotation tool.

Run:
    tools\\venv\\Scripts\\python.exe tools\\annotate_markers.py --split train

Shortcuts:
    1/L, 2/T, 3/X : select class
    left click     : place a default-size box at the cursor
    left drag      : draw a free box
    right click    : delete the box under the cursor
    wheel          : zoom at cursor
    middle drag    : pan (or hold Space + left drag)
    n / Right      : next frame (auto-save)
    p / Left       : previous frame
    u              : undo last box
    c              : clear all boxes on this frame
    d              : discard this frame (move to discarded/)
    f              : fit image to window
    g              : toggle 100 px grid
    h              : hide/show boxes
    [ / ]          : smaller / larger default box
    + / -          : zoom
    ?              : help
    Esc            : exit (auto-save)
"""
from __future__ import annotations

import argparse
import csv
import re
import shutil
import time
from pathlib import Path
from tkinter import Tk, Canvas, Label, StringVar, messagebox

from PIL import Image, ImageTk

ROOT = Path(__file__).resolve().parent.parent
CLASS_NAMES = {0: "L", 1: "T", 2: "X"}
CLASS_COLORS = {0: "#39d353", 1: "#4aa8ff", 2: "#ff5c5c"}


def natural_key(path: Path):
    return [int(p) if p.isdigit() else p.lower() for p in re.split(r"(\d+)", path.name)]


class MarkerAnnotator:
    def __init__(self, root: Tk, split: str, start: int, box_size: int):
        self.root = root
        self.split = split
        self.dataset = ROOT / "dataset" / "marker_dataset"
        self.image_dir = self.dataset / "images" / split
        self.label_dir = self.dataset / "labels" / split
        self.backup_dir = self.dataset / "labels_orig" / split
        self.discard_dir = self.dataset / "discarded" / split
        self.label_dir.mkdir(parents=True, exist_ok=True)
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        (self.discard_dir / "images").mkdir(parents=True, exist_ok=True)
        (self.discard_dir / "labels").mkdir(parents=True, exist_ok=True)

        self.files = sorted(self.image_dir.glob("*.jpg"), key=natural_key)
        if not self.files:
            raise SystemExit(f"no images in {self.image_dir}")
        self.index = max(0, min(start, len(self.files) - 1))
        self.box_size = max(20, min(400, box_size))
        self.img: Image.Image | None = None
        self.photo = None
        self.scale = 1.0
        self.offx = 0.0
        self.offy = 0.0
        self.boxes: list[dict] = []
        self.current_class = 0
        self.grid = False
        self.hidden = False
        self.backed_up: set[str] = set()
        self.dirty = False
        self.start_xy = None
        self.temp_id = None
        self.pan_from = None
        self.space_down = False
        self.cross_h = None
        self.cross_v = None

        self.root.title("sumaolin marker annotator")
        self.root.geometry("1500x940")
        self.canvas = Canvas(root, bg="#101010", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.status = StringVar()
        self.status_label = Label(root, textvariable=self.status, anchor="w",
                                  bg="#181818", fg="#d0d0d0",
                                  font=("Consolas", 10))
        self.status_label.pack(fill="x")

        self.bind_all_keys()
        self.canvas.bind("<ButtonPress-1>", self.on_left_down)
        self.canvas.bind("<B1-Motion>", self.on_left_move)
        self.canvas.bind("<ButtonRelease-1>", self.on_left_up)
        self.canvas.bind("<ButtonPress-2>", self.on_middle_down)
        self.canvas.bind("<B2-Motion>", self.on_middle_move)
        self.canvas.bind("<ButtonRelease-2>", self.on_middle_up)
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.canvas.bind("<MouseWheel>", self.on_wheel)
        self.canvas.bind("<Motion>", self.on_motion)
        self.canvas.bind("<Configure>", self.on_resize)
        self.root.bind("<KeyPress-space>", self.on_space_down)
        self.root.bind("<KeyRelease-space>", self.on_space_up)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.canvas.focus_set()
        self.root.after(50, self.load_current)

    def bind_all_keys(self):
        binds = {
            "1": lambda e: self.set_class(0),
            "2": lambda e: self.set_class(1),
            "3": lambda e: self.set_class(2),
            "l": lambda e: self.set_class(0),
            "t": lambda e: self.set_class(1),
            "x": lambda e: self.set_class(2),
            "n": lambda e: self.next_frame(),
            "Right": lambda e: self.next_frame(),
            "p": lambda e: self.prev_frame(),
            "Left": lambda e: self.prev_frame(),
            "d": lambda e: self.discard_frame(),
            "c": lambda e: self.clear_boxes(),
            "u": lambda e: self.undo(),
            "f": lambda e: self.fit(),
            "g": lambda e: self.toggle_grid(),
            "h": lambda e: self.toggle_hidden(),
            "plus": lambda e: self.zoom_center(1.2),
            "equal": lambda e: self.zoom_center(1.2),
            "minus": lambda e: self.zoom_center(1 / 1.2),
            "bracketleft": lambda e: self.set_box_size(-10),
            "bracketright": lambda e: self.set_box_size(10),
            "question": lambda e: self.show_help(),
            "Escape": lambda e: self.on_close(),
        }
        for key, fn in binds.items():
            self.root.bind(f"<KeyPress-{key}>", fn)
        self.root.bind("<Up>", lambda e: self.pan(0, 40))
        self.root.bind("<Down>", lambda e: self.pan(0, -40))

    # ---------- image / transform ----------
    def load_current(self):
        if not self.files:
            return
        path = self.files[self.index]
        self.img = Image.open(path).convert("RGB")
        self.boxes = self.load_boxes(path.stem)
        self.dirty = False
        self.temp_id = None
        self.fit()
        self.update_status()

    def fit(self):
        if not self.img:
            return
        cw = max(100, self.canvas.winfo_width())
        ch = max(100, self.canvas.winfo_height())
        iw, ih = self.img.size
        self.scale = min((cw - 20) / iw, (ch - 20) / ih)
        self.offx = (cw - iw * self.scale) / 2
        self.offy = (ch - ih * self.scale) / 2
        self.render()

    def render(self):
        if not self.img:
            return
        self.canvas.delete("all")
        iw, ih = self.img.size
        dw, dh = max(1, int(iw * self.scale)), max(1, int(ih * self.scale))
        disp = self.img.resize((dw, dh), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(disp)
        self.canvas.create_image(self.offx, self.offy, anchor="nw", image=self.photo)
        self.canvas.create_rectangle(self.offx, self.offy,
                                     self.offx + dw, self.offy + dh,
                                     outline="#444444", width=2)
        if self.grid and not self.hidden:
            self.draw_grid(iw, ih)
        if not self.hidden:
            for i, box in enumerate(self.boxes):
                x1, y1, x2, y2 = self.img_to_screen_box(box)
                color = CLASS_COLORS[box["cls"]]
                self.canvas.create_rectangle(x1, y1, x2, y2, outline=color, width=3)
                self.canvas.create_text(x1, max(self.offy + 8, y1 - 4),
                                        text=f"{CLASS_NAMES[box['cls']]} {i + 1}",
                                        anchor="sw", fill=color,
                                        font=("Consolas", 12, "bold"))
        self.cross_h = None
        self.cross_v = None

    def draw_grid(self, iw, ih):
        step = 100
        for gx in range(0, iw, step):
            sx = self.offx + gx * self.scale
            self.canvas.create_line(sx, self.offy, sx, self.offy + ih * self.scale,
                                    fill="#305080", dash=(2, 4))
            self.canvas.create_text(sx + 2, self.offy + 4, text=str(gx), anchor="nw",
                                    fill="#80a0ff", font=("Consolas", 8))
        for gy in range(0, ih, step):
            sy = self.offy + gy * self.scale
            self.canvas.create_line(self.offx, sy, self.offx + iw * self.scale, sy,
                                    fill="#305080", dash=(2, 4))
            self.canvas.create_text(self.offx + 4, sy + 2, text=str(gy), anchor="nw",
                                    fill="#80a0ff", font=("Consolas", 8))

    def img_to_screen(self, x, y):
        return self.offx + x * self.scale, self.offy + y * self.scale

    def screen_to_img(self, x, y):
        return (x - self.offx) / self.scale, (y - self.offy) / self.scale

    def img_to_screen_box(self, box):
        x1, y1 = self.img_to_screen(box["x1"], box["y1"])
        x2, y2 = self.img_to_screen(box["x2"], box["y2"])
        return x1, y1, x2, y2

    def clamp_box(self, x1, y1, x2, y2):
        if not self.img:
            return None
        iw, ih = self.img.size
        x1, x2 = sorted((max(0, min(iw - 1, x1)), max(0, min(iw - 1, x2))))
        y1, y2 = sorted((max(0, min(ih - 1, y1)), max(0, min(ih - 1, y2))))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        return x1, y1, x2, y2

    # ---------- mouse ----------
    def on_left_down(self, event):
        self.canvas.focus_set()
        if self.space_down:
            self.pan_from = (event.x, event.y, self.offx, self.offy)
            return
        self.start_xy = (event.x, event.y)
        self.temp_id = None

    def on_left_move(self, event):
        if self.pan_from:
            sx, sy, ox, oy = self.pan_from
            self.offx = ox + event.x - sx
            self.offy = oy + event.y - sy
            self.render()
            return
        if not self.start_xy:
            return
        x0, y0 = self.start_xy
        if self.temp_id:
            self.canvas.coords(self.temp_id, x0, y0, event.x, event.y)
        else:
            self.temp_id = self.canvas.create_rectangle(
                x0, y0, event.x, event.y,
                outline=CLASS_COLORS[self.current_class], width=2, dash=(4, 3))

    def on_left_up(self, event):
        if self.pan_from:
            self.pan_from = None
            return
        if not self.start_xy:
            return
        x0, y0 = self.start_xy
        self.start_xy = None
        if self.temp_id:
            self.canvas.delete(self.temp_id)
            self.temp_id = None
        moved = abs(event.x - x0) + abs(event.y - y0)
        if moved <= 5:
            ix, iy = self.screen_to_img(event.x, event.y)
            half = self.box_size / 2
            box = self.clamp_box(ix - half, iy - half, ix + half, iy + half)
        else:
            ix0, iy0 = self.screen_to_img(x0, y0)
            ix1, iy1 = self.screen_to_img(event.x, event.y)
            box = self.clamp_box(ix0, iy0, ix1, iy1)
        if box:
            self.boxes.append({"cls": self.current_class,
                               "x1": box[0], "y1": box[1],
                               "x2": box[2], "y2": box[3]})
            self.dirty = True
            self.save_current(write=True)
        self.render()
        self.update_status()

    def on_middle_down(self, event):
        self.pan_from = (event.x, event.y, self.offx, self.offy)
        self.canvas.configure(cursor="fleur")

    def on_middle_move(self, event):
        if not self.pan_from:
            return
        sx, sy, ox, oy = self.pan_from
        self.offx = ox + event.x - sx
        self.offy = oy + event.y - sy
        self.render()

    def on_middle_up(self, event):
        self.pan_from = None
        self.canvas.configure(cursor="")

    def on_right_click(self, event):
        ix, iy = self.screen_to_img(event.x, event.y)
        hit = None
        for idx, box in enumerate(self.boxes):
            if box["x1"] <= ix <= box["x2"] and box["y1"] <= iy <= box["y2"]:
                hit = idx
        if hit is not None:
            self.boxes.pop(hit)
            self.dirty = True
            self.save_current(write=True)
            self.render()
            self.update_status()
        else:
            self.set_status("right click: no box under cursor")

    def on_motion(self, event):
        if not self.img:
            return
        ix, iy = self.screen_to_img(event.x, event.y)
        counts = [0, 0, 0]
        for b in self.boxes:
            counts[b["cls"]] += 1
        self.set_status(
            f"{self.index + 1}/{len(self.files)} {self.files[self.index].name} | "
            f"cursor ({ix:.0f},{iy:.0f}) | L{counts[0]} T{counts[1]} X{counts[2]} | "
            f"class {CLASS_NAMES[self.current_class]} | box {self.box_size}px | "
            f"zoom {self.scale:.2f}x | grid {'on' if self.grid else 'off'}"
        )
        if self.cross_h:
            self.canvas.delete(self.cross_h)
        if self.cross_v:
            self.canvas.delete(self.cross_v)
        self.cross_h = self.canvas.create_line(event.x, 0, event.x,
                                               self.canvas.winfo_height(),
                                               fill="#ffcc00", dash=(1, 5))
        self.cross_v = self.canvas.create_line(0, event.y,
                                               self.canvas.winfo_width(), event.y,
                                               fill="#ffcc00", dash=(1, 5))

    def on_wheel(self, event):
        if not self.img:
            return
        factor = 1.2 if event.delta > 0 else 1 / 1.2
        ix, iy = self.screen_to_img(event.x, event.y)
        self.scale = max(0.05, min(4.0, self.scale * factor))
        self.offx = event.x - ix * self.scale
        self.offy = event.y - iy * self.scale
        self.render()
        self.update_status()

    def on_resize(self, event):
        if self.img and not self.start_xy:
            self.render()

    def on_space_down(self, event):
        self.space_down = True
        self.canvas.configure(cursor="fleur")

    def on_space_up(self, event):
        self.space_down = False
        self.canvas.configure(cursor="")

    # ---------- keys ----------
    def set_class(self, cls):
        self.current_class = cls
        self.update_status()

    def set_box_size(self, delta):
        self.box_size = max(20, min(400, self.box_size + delta))
        self.update_status()

    def zoom_center(self, factor):
        cw = self.canvas.winfo_width() / 2
        ch = self.canvas.winfo_height() / 2
        ix, iy = self.screen_to_img(cw, ch)
        self.scale = max(0.05, min(4.0, self.scale * factor))
        self.offx = cw - ix * self.scale
        self.offy = ch - iy * self.scale
        self.render()
        self.update_status()

    def pan(self, dx, dy):
        self.offx += dx
        self.offy += dy
        self.render()

    def next_frame(self):
        if self.dirty:
            self.save_current(write=True)
        if self.index + 1 >= len(self.files):
            self.set_status("last frame")
            return
        self.index += 1
        self.load_current()

    def prev_frame(self):
        if self.dirty:
            self.save_current(write=True)
        self.index = max(0, self.index - 1)
        self.load_current()

    def clear_boxes(self):
        self.boxes = []
        self.dirty = True
        self.save_current(write=True)
        self.render()
        self.update_status()

    def undo(self):
        if self.boxes:
            self.boxes.pop()
            self.dirty = True
            self.save_current(write=True)
            self.render()
            self.update_status()

    def toggle_grid(self):
        self.grid = not self.grid
        self.render()
        self.update_status()

    def toggle_hidden(self):
        self.hidden = not self.hidden
        self.render()
        self.update_status()

    # ---------- labels ----------
    def load_boxes(self, stem):
        path = self.label_dir / f"{stem}.txt"
        boxes = []
        if path.exists():
            iw, ih = self.img.size
            for line in path.read_text(encoding="utf-8").splitlines():
                p = line.split()
                if len(p) != 5:
                    continue
                cls = int(p[0])
                cx, cy, bw, bh = map(float, p[1:])
                boxes.append({
                    "cls": cls,
                    "x1": (cx - bw / 2) * iw,
                    "y1": (cy - bh / 2) * ih,
                    "x2": (cx + bw / 2) * iw,
                    "y2": (cy + bh / 2) * ih,
                })
        return boxes

    def save_current(self, write=False):
        if not self.img or not self.files:
            return
        stem = self.files[self.index].stem
        path = self.label_dir / f"{stem}.txt"
        if write:
            if stem not in self.backed_up and path.exists():
                shutil.copy2(path, self.backup_dir / path.name)
                self.backed_up.add(stem)
            iw, ih = self.img.size
            lines = []
            for box in self.boxes:
                cx = (box["x1"] + box["x2"]) / 2 / iw
                cy = (box["y1"] + box["y2"]) / 2 / ih
                bw = abs(box["x2"] - box["x1"]) / iw
                bh = abs(box["y2"] - box["y1"]) / ih
                lines.append(f"{box['cls']} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
            path.write_text("\n".join(lines) + ("\n" if lines else ""),
                            encoding="utf-8")
            self.dirty = False

    def discard_frame(self):
        if not self.img or not self.files:
            return
        path = self.files[self.index]
        stem = path.stem
        label = self.label_dir / f"{stem}.txt"
        dst_img = self.discard_dir / "images" / f"{self.split}__{path.name}"
        dst_lbl = self.discard_dir / "labels" / f"{self.split}__{label.name}"
        shutil.move(str(path), str(dst_img))
        if label.exists():
            shutil.move(str(label), str(dst_lbl))
        record = self.dataset / "discarded.csv"
        new = not record.exists()
        with record.open("a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "split", "stem", "image", "label"])
            w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), self.split, stem,
                        str(dst_img), str(dst_lbl)])
        self.files.pop(self.index)
        if not self.files:
            self.img = None
            self.render()
            self.set_status("all frames processed")
            return
        self.index = min(self.index, len(self.files) - 1)
        self.load_current()

    def update_status(self):
        if not self.img or not self.files:
            return
        counts = [0, 0, 0]
        for b in self.boxes:
            counts[b["cls"]] += 1
        self.set_status(
            f"{self.index + 1}/{len(self.files)} {self.files[self.index].name} | "
            f"L{counts[0]} T{counts[1]} X{counts[2]} | "
            f"class {CLASS_NAMES[self.current_class]} | box {self.box_size}px | "
            f"zoom {self.scale:.2f}x | grid {'on' if self.grid else 'off'}"
        )

    def set_status(self, text):
        self.status.set(text)

    def show_help(self):
        messagebox.showinfo(
            "Shortcuts",
            "1/L  L class      2/T  T class      3/X  X class\n"
            "left click  place box at cursor\n"
            "left drag   free box\n"
            "right click delete box under cursor\n"
            "wheel       zoom at cursor\n"
            "middle drag / Space+left drag  pan\n"
            "n / Right   next frame (auto save)\n"
            "p / Left    previous frame\n"
            "u           undo last box\n"
            "c           clear all boxes on frame\n"
            "d           discard frame -> discarded/\n"
            "f           fit window\n"
            "g           grid\n"
            "h           hide/show boxes\n"
            "[ / ]       smaller/larger default box\n"
            "+ / -       zoom\n"
            "?           this help\n"
            "Esc         exit (auto save)",
        )

    def on_close(self):
        try:
            if self.dirty:
                self.save_current(write=True)
        except Exception:
            pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", default="train", choices=("train", "val", "test"))
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--box-size", type=int, default=120)
    args = parser.parse_args()
    root = Tk()
    MarkerAnnotator(root, args.split, args.start, args.box_size)
    root.mainloop()


if __name__ == "__main__":
    main()
