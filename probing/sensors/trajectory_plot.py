import math
from pathlib import Path


class ProbeTrajectoryPlotter:
    """Record stage-end TCP points and save a lightweight 3D trajectory plot."""

    def __init__(self, node, output_path, primitive, base_frame, tip_link):
        self.node = node
        self.output_path = Path(output_path)
        self.primitive = primitive
        self.base_frame = base_frame
        self.tip_link = tip_link
        self.records = []
        self._init_tf()

        print(
            "Trajectory plot enabled: "
            f"path={self.output_path}, tip_link={self.tip_link}."
        )

    @classmethod
    def from_args(cls, node, args, primitive):
        if not args.trajectory_plot_path:
            return None

        return cls(
            node=node,
            output_path=args.trajectory_plot_path,
            primitive=primitive,
            base_frame=args.base_frame,
            tip_link=args.tip_link,
        )

    def _init_tf(self):
        try:
            from tf2_ros import Buffer, TransformListener
        except ModuleNotFoundError:
            self.tf_buffer = None
            self.tf_listener = None
            return

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=False,
        )

    def warmup(self, duration_s=0.25):
        import rclpy
        import time

        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def capture(self, stage, event="sample", command_pose=None):
        if event != "done":
            return

        pose = self._lookup_tf_pose()
        source = "tf"
        if pose is None and command_pose is not None:
            pose = command_pose[:3]
            source = "commanded_pose_fallback"

        if pose is None:
            print(
                "WARNING: Could not record trajectory point for "
                f"{stage}; no TF or commanded pose is available."
            )
            return

        command_xyz = command_pose[:3] if command_pose is not None else None
        self.records.append(
            {
                "stage": stage,
                "event": event,
                "x": float(pose[0]),
                "y": float(pose[1]),
                "z": float(pose[2]),
                "source": source,
                "command_xyz": command_xyz,
            }
        )

    def _lookup_tf_pose(self):
        if self.tf_buffer is None:
            return None

        try:
            from rclpy.time import Time

            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tip_link,
                Time(),
            )
        except Exception:
            return None

        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    def close(self):
        if not self.output_path:
            return
        if not self.records:
            print("WARNING: No trajectory stage points were recorded.")
            return

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        suffix = self.output_path.suffix.lower()
        if suffix == ".png":
            try:
                self._save_png()
            except ModuleNotFoundError as exc:
                fallback_path = self.output_path.with_suffix(".svg")
                print(
                    "WARNING: matplotlib is not available "
                    f"({exc.name}); saving SVG fallback to {fallback_path}."
                )
                self.output_path = fallback_path
                self._save_svg()
        else:
            self._save_svg()

        print(
            "Trajectory plot saved: "
            f"{self.output_path} ({len(self.records)} stage points)."
        )

    def _save_png(self):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        xs = [record["x"] for record in self.records]
        ys = [record["y"] for record in self.records]
        zs = [record["z"] for record in self.records]

        fig = plt.figure(figsize=(7.0, 5.5))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(xs, ys, zs, marker="o", linewidth=2.0, label="actual TCP")

        command_points = [
            record["command_xyz"]
            for record in self.records
            if record["command_xyz"] is not None
        ]
        if command_points:
            cmd_xs = [point[0] for point in command_points]
            cmd_ys = [point[1] for point in command_points]
            cmd_zs = [point[2] for point in command_points]
            ax.plot(
                cmd_xs,
                cmd_ys,
                cmd_zs,
                linestyle="--",
                marker="x",
                linewidth=1.2,
                label="commanded pose",
            )

        for idx, record in enumerate(self.records, start=1):
            ax.text(record["x"], record["y"], record["z"], f"{idx}:{record['stage']}")

        ax.set_title(f"{self.primitive} {self.tip_link} stage trajectory")
        ax.set_xlabel(f"x in {self.base_frame} (m)")
        ax.set_ylabel(f"y in {self.base_frame} (m)")
        ax.set_zlabel(f"z in {self.base_frame} (m)")
        ax.legend()
        ax.view_init(elev=24, azim=-58)
        fig.tight_layout()
        fig.savefig(self.output_path, dpi=160)
        plt.close(fig)

    def _save_svg(self):
        projected = [self._project(record["x"], record["y"], record["z"]) for record in self.records]
        cmd_projected = [
            self._project(*record["command_xyz"])
            if record["command_xyz"] is not None
            else None
            for record in self.records
        ]

        all_points = list(projected) + [point for point in cmd_projected if point]
        raw_min_u = min(point[0] for point in all_points)
        raw_max_u = max(point[0] for point in all_points)
        raw_min_v = min(point[1] for point in all_points)
        raw_max_v = max(point[1] for point in all_points)

        width = 900
        height = 650
        pad = 80
        span_u = max(raw_max_u - raw_min_u, 0.08)
        span_v = max(raw_max_v - raw_min_v, 0.08)
        center_u = 0.5 * (raw_min_u + raw_max_u)
        center_v = 0.5 * (raw_min_v + raw_max_v)
        min_u = center_u - 0.5 * span_u
        min_v = center_v - 0.5 * span_v
        scale = min((width - 2 * pad) / span_u, (height - 2 * pad) / span_v)

        def to_canvas(point):
            u, v = point
            x = pad + (u - min_u) * scale
            y = height - pad - (v - min_v) * scale
            return (x, y)

        canvas_points = [to_canvas(point) for point in projected]
        canvas_cmd_points = [
            to_canvas(point) if point is not None else None for point in cmd_projected
        ]

        actual_polyline = self._polyline(canvas_points)
        command_polyline = self._polyline(
            [point for point in canvas_cmd_points if point is not None]
        )
        axes = self._svg_axes(to_canvas)

        labels = []
        for idx, (record, point) in enumerate(
            zip(self.records, canvas_points),
            start=1,
        ):
            x, y = point
            labels.append(
                f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#0f766e" />'
            )
            labels.append(
                '<text '
                f'x="{x + 8:.1f}" y="{y - 8:.1f}" '
                'font-family="Arial, sans-serif" font-size="13" '
                'fill="#111827">'
                f'{idx}:{self._escape(record["stage"])}</text>'
            )

        command_marks = []
        for point in canvas_cmd_points:
            if point is None:
                continue
            x, y = point
            command_marks.append(
                '<path '
                f'd="M {x - 5:.1f} {y - 5:.1f} L {x + 5:.1f} {y + 5:.1f} '
                f'M {x + 5:.1f} {y - 5:.1f} L {x - 5:.1f} {y + 5:.1f}" '
                'stroke="#dc2626" stroke-width="2" />'
            )

        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#ffffff" />
  <text x="40" y="42" font-family="Arial, sans-serif" font-size="24" font-weight="700" fill="#111827">{self._escape(self.primitive)} {self._escape(self.tip_link)} stage trajectory</text>
  <text x="40" y="68" font-family="Arial, sans-serif" font-size="14" fill="#4b5563">Projected 3D view in {self._escape(self.base_frame)}. Solid teal = actual TCP TF. Dashed red = commanded pose.</text>
  {axes}
  <polyline points="{command_polyline}" fill="none" stroke="#dc2626" stroke-width="2" stroke-dasharray="8 6" />
  <polyline points="{actual_polyline}" fill="none" stroke="#0f766e" stroke-width="3" />
  {''.join(command_marks)}
  {''.join(labels)}
</svg>
'''
        self.output_path.write_text(svg)

    @staticmethod
    def _project(x, y, z):
        azimuth = math.radians(-45.0)
        elevation = math.radians(24.0)
        u = math.cos(azimuth) * x - math.sin(azimuth) * y
        v = (
            math.sin(elevation) * math.sin(azimuth) * x
            + math.sin(elevation) * math.cos(azimuth) * y
            + math.cos(elevation) * z
        )
        return (u, v)

    @staticmethod
    def _polyline(points):
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in points)

    def _svg_axes(self, to_canvas):
        origin = (0.0, 0.0, min(record["z"] for record in self.records))
        axes = (
            ("x", "#64748b", (0.05, 0.0, 0.0)),
            ("y", "#64748b", (0.0, 0.05, 0.0)),
            ("z", "#64748b", (0.0, 0.0, 0.05)),
        )
        lines = []
        ox, oy = to_canvas(self._project(*origin))
        for label, color, delta in axes:
            end = (
                origin[0] + delta[0],
                origin[1] + delta[1],
                origin[2] + delta[2],
            )
            ex, ey = to_canvas(self._project(*end))
            lines.append(
                f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" '
                f'stroke="{color}" stroke-width="1.5" />'
            )
            lines.append(
                f'<text x="{ex + 5:.1f}" y="{ey - 5:.1f}" '
                'font-family="Arial, sans-serif" font-size="12" '
                f'fill="{color}">{label}</text>'
            )
        return "".join(lines)

    @staticmethod
    def _escape(value):
        return (
            str(value)
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
        )
