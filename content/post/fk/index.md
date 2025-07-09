---
title: Forward Kinematics
summary: Forward kinematics for robotic systems
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - Kinematics
  - Control
  - Theory
---

Forward kinematics (FK) allow us to determine the position of the end-effector based on the positions of the joints of the robot.

## Recommended Pre-Reading

Before reading this, you should have some familiarity with matrices and algebra, and you should read the discuss on [transformations]({{< relref "/post/transformations/" >}}).

---

A robot is modelled as a series of links and joints.

{{% callout note %}}
Joint {{< math >}}$i${{< /math >}} separates links {{< math >}}$i-1${{< /math >}} and {{< math >}}$i${{< /math >}}
{{% /callout %}}

The base coordinate system is frame {{< math >}}$0${{< /math >}}, and the end-effector is frame {{< math >}}$n${{< /math >}}. It is natural to express the motion of link {{< math >}}$i${{< /math >}} w.r.t. link {{< math >}}$i-1${{< /math >}}.

FK express motion of an end-effector by carrying out a sequence of transformations starting at the base frame and progressing through links until reaching the end-effector.

{{< math >}}

$$
^0_nT=~^0_1T~^1_2T\dots~^{n-2}_{n-1}T~^{n-1}_nT
$$

{{< /math >}}

Where {{< math >}}$^{i-1}_iT${{< /math >}} is a homogeneous transformation matrix.

![Robot Link Diagram](link_diagram.png)

Expressing FK using a kinematic chain makes sense because it is relatively easy to express the motion of one joint w.r.t. another. It is considerably more difficult to directly express the end-effector position w.r.t. the base without the use of a kinematic chain.

## Observations on Robot Structure

The structure of a homogeneous transformation matrix is complicated and dependent on the origins of the consitituent reference frames, therefore it also depends on the structure of the robot.

It is possible to reduce these concerns by observing that for most robots:

- Most links are not rotated in 3D from the previous one
- Most links are not offset from another in 3D

![Robot Structure](robot_structure.png)

## Denavit-Hartenberg Method

A the Denavit-Hartenberg (DH) method is a semi-systematic method for obtaining the FK equations for robots. It guides the choosing of appropriate frames and effectively determines the T-matrix for each link.

It incorporates the simplifications outlined above which is useful as it is effectively universally applicable and gives a common framework for kinematic equations.

{{% callout note %}}
The DH method assumes each joint is either revolute or prismatic. The joint variable for revolute joints is the angle, {{< math >}}$\theta${{< /math >}}. For prismatic joints, the joint variable is the linear actuator distance, {{< math >}}$d${{< /math >}}.
{{% /callout %}}

Reference frames are established by following these rules:

1. Each {{< math >}}$Z${{< /math >}} axis passes through a joint
2. The {{< math >}}$X_i${{< /math >}} axis is perpendicular to the {{< math >}}$Z_{i-1}${{< /math >}} axis
3. The {{< math >}}$X_i${{< /math >}} axis intersects the {{< math >}}$Z_{i-1}${{< /math >}} axis
4. The {{< math >}}$Y_i${{< /math >}} axis is determined by the choice of {{< math >}}$X_i${{< /math >}} and {{< math >}}$Z_i${{< /math >}} (RH system)

### Homogeneous Transformations

Once the reference frames are established, the DH method gives a systematic method for deriving the T-matrices for each link

The protocol for moving reference frames in the DH convention is:

1. Rotate {{< math >}}$\theta${{< /math >}} around {{< math >}}$Z_{i-1}${{< /math >}}
2. Translate {{< math >}}$d${{< /math >}} along {{< math >}}$Z_{i-1}${{< /math >}}
3. Translate {{< math >}}$a${{< /math >}} along {{< math >}}$X_i${{< /math >}}
4. Rotate {{< math >}}$\alpha${{< /math >}} along {{< math >}}$X_i${{< /math >}}

{{< math >}}

$$
^{i-1}_iT=~T_{\text{rot},Z_i-1}~~T_{\text{trans},Z_i-1}~~T_{\text{trans},X_i}~~T_{\text{rot},X_i}
$$

{{< /math >}}

For each joint only one joint parameter is assumed ({{< math >}}$\theta${{< /math >}} or {{< math >}}$d${{< /math >}}).

### Axis Notation

The DH method places the {{< math >}}$i-1${{< /math >}}th frame on the {{< math >}}$i${{< /math >}}th joint.

![DH Axis Notation](axis_notation.png)

### Frame Uniqueness

The DH method does not choose frames uniquely; {{< math >}}$X_i\perp Z_{i-1}${{< /math >}} and they must intersect.

![DH Frame Uniqueness](frame_uniqueness.png)

We can choose the “up” or “down” nature of {{< math >}}$Z_i${{< /math >}} and the “forward” or “backwards” nature of {{< math >}}$X_i${{< /math >}}; but we should choose the most intuitive orientation. If {{< math >}}$Z_i${{< /math >}} and {{< math >}}$Z_{i-1}${{< /math >}} axes are coincident, then the {{< math >}}$X_i${{< /math >}} direction is completely free; choose the simplest choices to zero some entries in the T-matrix. If {{< math >}}$Z_i${{< /math >}} and {{< math >}}$Z_{i-1}${{< /math >}} are parallel, {{< math >}}$O_i${{< /math >}} is arbitrary; choose the simplest choice to zero some entries in the T-matrix.

### Method Summary

1. Assume a robot has {{< math >}}$n${{< /math >}} joints and {{< math >}}$n+1${{< /math >}} links
2. Joint {{< math >}}$i${{< /math >}} connects link {{< math >}}$i-1${{< /math >}} and {{< math >}}$i${{< /math >}}
3. The {{< math >}}$i-1${{< /math >}}th reference frame {{< math >}}$\mathcal{F}_{i-1}${{< /math >}} is located on the joint {{< math >}}$i${{< /math >}}
4. The {{< math >}}$Z_{i-1}${{< /math >}} axis passes through joint {{< math >}}$i${{< /math >}}
5. Each joint is either prismatic ({{< math >}}$d_i${{< /math >}} variable) or revolute ({{< math >}}$\theta_i${{< /math >}} variable)
6. The base frame is assigned first: this is essentially arbitrary
7. Each reference frame is determined sequentially such that {{< math >}}$X_i\perp Z_{i-1}${{< /math >}}
8. Determine the 4 DH parameters
   1. {{< math >}}$\theta_i${{< /math >}}: joint angle between link {{< math >}}$i-1${{< /math >}} and link {{< math >}}$i${{< /math >}} (the angle around {{< math >}}$Z_{i-1})${{< /math >}}
   2. {{< math >}}$d_i${{< /math >}}: link offset is the distance alone {{< math >}}$Z_{i-1}${{< /math >}} until the intersection with {{< math >}}$X_i${{< /math >}}
   3. {{< math >}}$a_i${{< /math >}}: link length is the distance along {{< math >}}$X_i$$Z_{i-1}${{< /math >}} and {{< math >}}$Z_i${{< /math >}}
   4. {{< math >}}$\alpha_i${{< /math >}}: twist offset is the angle between {{< math >}}$Z_{i-1}-X_{i-1}${{< /math >}} planes and the {{< math >}}$Z_i-X_i${{< /math >}} planes
