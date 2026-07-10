import csv
from pathlib import Path

import mujoco
import numpy as np


class MujocoBackend:
    """Small MuJoCo wrapper used by the probing primitives."""

    def __init__(self, config):
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(config.model_path))
        self.data = mujoco.MjData(self.model)

        self.home_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY, "home"
        )
        self.tip_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "spatula_tip"
        )
        self.sample_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "sample_geom"
        )
        self.sample_geom_ids = set()
        if self.sample_geom_id >= 0:
            self.sample_geom_ids.add(self.sample_geom_id)
        for geom_id in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
            )
            if geom_name and geom_name.startswith("sample_particle"):
                self.sample_geom_ids.add(geom_id)

        if not self.sample_geom_ids:
            raise ValueError(
                "No sample geom found. Expected 'sample_geom' or geoms "
                "named 'sample_particle*'."
            )

        self.spatula_geom_ids = {
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, "spatula_handle"
            ),
            mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, "spatula_blade"
            ),
        }

    def reset_robot(self):
        # mj_resetData keeps free bodies, such as sand particles, at their
        # XML initial poses. A full keyframe reset would zero unspecified
        # freejoint qpos values and move particles to the world origin.
        mujoco.mj_resetData(self.model, self.data)
        if self.home_id >= 0:
            home_qpos = self.model.key_qpos[self.home_id]
            home_ctrl = self.model.key_ctrl[self.home_id]
            self.data.qpos[:9] = home_qpos[:9]
            self.data.ctrl[:8] = home_ctrl[:8]
        self.data.qpos[7:9] = self.config.gripper_position_m
        self.data.ctrl[7] = self.config.gripper_control
        mujoco.mj_forward(self.model, self.data)

    def keep_gripper_closed(self):
        self.data.ctrl[7] = self.config.gripper_control

    def home_q(self):
        return self.data.qpos[:7].copy()

    def tip_position(self):
        return self.data.site_xpos[self.tip_id].copy()

    def tip_rotation(self):
        return self.data.site_xmat[self.tip_id].reshape(3, 3).copy()

    def sample_top(self):
        tops = []
        for geom_id in self.sample_geom_ids:
            geom_type = self.model.geom_type[geom_id]
            geom_size = self.model.geom_size[geom_id]

            if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                half_height = geom_size[1]
            elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                half_height = geom_size[0]
            else:
                half_height = geom_size[2]

            tops.append(self.data.geom_xpos[geom_id][2] + half_height)

        return max(tops)

    def solve_position_ik(self, start_q, target_position):
        self.data.qpos[:7] = start_q.copy()
        mujoco.mj_forward(self.model, self.data)

        for _ in range(500):
            error = target_position - self.tip_position()
            if np.linalg.norm(error) < 0.0001:
                break

            jacobian = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(
                self.model, self.data, jacobian, None, self.tip_id
            )

            arm_jacobian = jacobian[:, :7]
            joint_change = np.linalg.pinv(arm_jacobian) @ error
            self.data.qpos[:7] += 0.2 * joint_change
            mujoco.mj_forward(self.model, self.data)

        return self.data.qpos[:7].copy()

    def solve_pose_ik(self, start_q, target_position, target_rotation):
        self.data.qpos[:7] = start_q.copy()
        mujoco.mj_forward(self.model, self.data)

        damping = 0.01

        for _ in range(1000):
            position_error = target_position - self.tip_position()
            rotation_error = orientation_error(
                target_rotation, self.tip_rotation()
            )
            error = np.concatenate([position_error, rotation_error])

            if (
                np.linalg.norm(position_error) < 0.0001
                and np.linalg.norm(rotation_error) < 0.001
            ):
                break

            position_jacobian = np.zeros((3, self.model.nv))
            rotation_jacobian = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(
                self.model,
                self.data,
                position_jacobian,
                rotation_jacobian,
                self.tip_id,
            )

            jacobian = np.vstack(
                [position_jacobian[:, :7], rotation_jacobian[:, :7]]
            )
            inverse = np.linalg.inv(
                jacobian @ jacobian.T + damping**2 * np.eye(6)
            )
            joint_change = jacobian.T @ inverse @ error
            joint_change = np.clip(joint_change, -0.05, 0.05)

            self.data.qpos[:7] += 0.2 * joint_change
            mujoco.mj_forward(self.model, self.data)

        return self.data.qpos[:7].copy()

    def sample_contact_force(self):
        total_normal_force = 0.0
        contact_count = 0
        contact_force = np.zeros(6)

        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]

            involves_sample = (
                contact.geom1 in self.sample_geom_ids
                or contact.geom2 in self.sample_geom_ids
            )
            involves_spatula = (
                contact.geom1 in self.spatula_geom_ids
                or contact.geom2 in self.spatula_geom_ids
            )

            if involves_sample and involves_spatula:
                mujoco.mj_contactForce(
                    self.model, self.data, contact_index, contact_force
                )
                total_normal_force += abs(contact_force[0])
                contact_count += 1

        return total_normal_force, contact_count

    def save_records(self, records, output_path):
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with output_path.open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "time_s",
                    "primitive",
                    "stage",
                    "tool_geometry",
                    "angle_deg",
                    "tip_x_m",
                    "tip_y_m",
                    "tip_z_m",
                    "gap_mm",
                    "contact_count",
                    "contact_force_n",
                ]
            )
            writer.writerows(records)


def interpolate(start_q, target_q, progress):
    progress = np.clip(progress, 0.0, 1.0)
    smooth = progress * progress * (3.0 - 2.0 * progress)
    return start_q + smooth * (target_q - start_q)


def rotation_y(angle):
    cosine = np.cos(angle)
    sine = np.sin(angle)
    return np.array(
        [
            [cosine, 0.0, sine],
            [0.0, 1.0, 0.0],
            [-sine, 0.0, cosine],
        ]
    )


def orientation_error(target_rotation, current_rotation):
    error_matrix = target_rotation @ current_rotation.T
    return 0.5 * np.array(
        [
            error_matrix[2, 1] - error_matrix[1, 2],
            error_matrix[0, 2] - error_matrix[2, 0],
            error_matrix[1, 0] - error_matrix[0, 1],
        ]
    )
