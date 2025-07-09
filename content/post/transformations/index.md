---
title: Transformations
summary: Transformations for robotic systems
date: 2025-01-01
authors:
  - Toby Godfrey
tags:
  - Kinematics
  - Control
  - Theory
---

This guide will provide a brief overview on how reference frames transform as we move through a kinematic chain. These concepts are necessary before moving on to more advanced kinematics guides.

{{% callout note %}}
For reasons you will see later, this guide will focus on manipulator robots, but the ideas are transferrable to other morphologies.
{{% /callout %}}

## Recommended Pre-Reading

Before reading this, you should have some familiarity with matrices and algebra.

---

## Kinematic Chains

Robot manipulators (arms) can be thought of chains of reference frames, or kinematic chains. The transformation for a reference frame is entirely dependent on the previous frame.

![Kinematic Chain](kinematic_chain.png)

It is easy to determine the {{< math >}}$i${{< /math >}}th joint position w.r.t. the {{< math >}}$i-1${{< /math >}}th reference frame, therefore the end-effector position is determined by traversing the chain:

{{< math >}}
$$ {}^0_nT={}^0_1T\,{}^1_2T\,\cdots\,{}^{n-1}\_nT $$
{{< /math >}}

Where {{< math >}}$^{i-1}_iT${{< /math >}} is the transformation matrix from link {{< math >}}$i-1${{< /math >}} to link {{< math >}}$i${{< /math >}}. In other words the rotation and displacement of frame {{< math >}}$i${{< /math >}} w.r.t. frame {{< math >}}$i-1${{< /math >}}

## 2D Transformations

To begin, we will look at only two dimensions. Once we have gone over the basics, we can extend to the three dimensions of the real world.

### Translation

Consider the below image.

![2D Translation Example](2d_translation.png)

There exists some particle {{< math >}}$P${{< /math >}} with two coordinate frames {{< math >}}$\{A\}=(x_a, y_a)${{< /math >}} and {{< math >}}$\{B\}=(x_b,y_b)${{< /math >}}. {{< math >}}$^AP_{B,\text{ORIG}}${{< /math >}} is the origin of {{< math >}}$\{B\}${{< /math >}} w.r.t. {{< math >}}$\{A\}${{< /math >}}. The position of the particle in frame {{< math >}}$Q${{< /math >}} is defined as {{< math >}}$^QP${{< /math >}}.

If we know the position of the particle in frame {{< math >}}$B${{< /math >}}, we can find the position of {{< math >}}$P${{< /math >}} in frame {{< math >}}$A${{< /math >}}, {{< math >}}$^AP=~^AP_{B,\text{ORIG}}+~^BP${{< /math >}}, where {{< math >}}$^AP_{B,\text{ORIG}}${{< /math >}} is a translational mapping from {{< math >}}$\{A\}${{< /math >}} to {{< math >}}$\{B\}${{< /math >}}.

### Rotation

We can use a similar approach for calculating the rotation between two reference frames.

![2D Rotation Example](2d_rotation.png)

Firstly, we must define our two frames,{{< math >}}$\{A\}${{< /math >}} and {{< math >}}$\{B\}${{< /math >}}. The position vector w.r.t. {{< math >}}$\{A\}${{< /math >}} is {{< math >}}$^AP=d\begin{bmatrix}\cos\theta_a \\ \sin\theta_a\end{bmatrix}${{< /math >}}; and for {{< math >}}$\{B\}${{< /math >}}, {{< math >}}$^BP=d\begin{bmatrix}\cos\theta_b \\ \sin\theta_b\end{bmatrix}${{< /math >}}.

{{% callout note %}}
It is worth noting that {{< math >}}$\theta_a=\alpha+\theta_b$${{< /math >}}.
{{% /callout %}}

We can use this information to find an equation to rotate the position vector between frames.

{{< math >}}

$$
\begin{align*}
  ^AP&=d\begin{bmatrix}\cos(\theta_b+\alpha)\\\sin(\theta_b+\alpha)\end{bmatrix}\\&=d\begin{bmatrix}
  \cos\theta_b\cos\alpha-\sin\theta_b\sin\alpha\\
  \sin\theta_b\cos\alpha+\cos\theta_b\sin\alpha
  \end{bmatrix}\\&=
  \begin{bmatrix}
  \cos\alpha & -\sin\alpha\\\sin\alpha&\cos\alpha
  \end{bmatrix}d\begin{bmatrix}\cos\theta_b\\\sin\theta_b\end{bmatrix}\\
  &=
  \begin{bmatrix}
  \cos\alpha & -\sin\alpha\\\sin\alpha&\cos\alpha
  \end{bmatrix}~^BP
\end{align*}
$$

{{< /math >}}

The matrix {{< math >}}$^A_BR=\begin{bmatrix}\cos\alpha&-\sin\alpha\\\sin\alpha&\cos\alpha\end{bmatrix}${{< /math >}} is a rotation matrix for the rotation of the axes of {{< math >}}$\{B\}${{< /math >}} by {{< math >}}$\alpha${{< /math >}} (anti-clockwise) from {{< math >}}$\{A\}${{< /math >}}

Succinctly, we can write this as {{< math >}}$^AP=~^A_BR~^BP$${{< /math >}}.

### Homogeneous Transformations

We can combine translations and rotations into a single operation called a homogeneous transformation.

If a coordinate frame {{< math >}}$\{B\}${{< /math >}} is translated by the vector {{< math >}}$^AP_{B,\text{ORIG}}${{< /math >}} from {{< math >}}$\{A\}${{< /math >}} and also rotated by an angle {{< math >}}$\alpha${{< /math >}} anti-clockwise, if we know the position of a particle in {{< math >}}$\{B\}${{< /math >}} we can express it in {{< math >}}$\{A\}${{< /math >}}

{{< math >}}

$$
^AP=~^AP_{B,\text{ORIG}}+~^A_BR~~^BP
$$

{{< /math >}}

This can be expressed in matrix form as

{{< math >}}

$$
^AP=\begin{bmatrix}^A_BR&^AP_{B,\text{ORIG}}\end{bmatrix}\begin{bmatrix}^BP\\1\end{bmatrix}
$$

{{< /math >}}

This above matrix is not square and so not invertible, so we can augment it as

{{< math >}}

$$
\begin{bmatrix}^AP\\1\end{bmatrix}=\begin{bmatrix}^A_BR&^AP_{B,\text{ORIG}}\\0&1\end{bmatrix}\begin{bmatrix}^BP\\1\end{bmatrix}
$$

{{< /math >}}

By defining {{< math >}}$^A\tilde{P}:=\begin{bmatrix}^AP\\ 1\end{bmatrix}${{< /math >}} and {{< math >}}$^B\tilde{P}:=\begin{bmatrix}^BP\\ 1\end{bmatrix}${{< /math >}} we have {{< math >}}$^A\tilde{P}=~^A_BT~~^B\tilde{P}${{< /math >}} where {{< math >}}$^A_BT=\begin{bmatrix}^A_BR&^AP_{B,\text{ORIG}}\\0&1\end{bmatrix}${{< /math >}}

{{% callout note %}}
**Homogeneous Transformation:** the matrix {{< math >}}$^A_BT${{< /math >}} is the so-called homogeneous transformation matrix which expresses the transformation between the frame {{< math >}}$\{B\}${{< /math >}} and {{< math >}}$\{A\}${{< /math >}}.
{{% /callout %}}

The matrices of {{< math >}}${}^A_BT${{< /math >}} can be expanded to produce the matrix

{{< math >}}

$$
{}^A_BT=\left[ \begin{array}{c c|c}
\cos \alpha & -\sin \alpha & ({}^A P_{B,\text{ORIG}})_x \\
\sin \alpha & \cos \alpha & ({}^A P_{B,\text{ORIG}})_y \\
\hline
0 & 0 & 1 \\
\end{array} \right]
$$

{{< /math >}}

## 3D Transformations

The method for 3D transformations is similar to that in 2D, just with an extra dimension!

### Translation

Consider the below image.

![3D Translation Example](3d_translation.png)

For some particle {{< math >}}$P${{< /math >}} there are two coordinate frames:

{{< math >}}

$$
\begin{align*}
\{A\}&=(x_a,y_a,z_a)\\
\{B\}&=(x_b, y_b, z_b)
\end{align*}
$$

{{< /math >}}

Similarly, {{< math >}}$^AP_{B,\text{ORIG}}${{< /math >}} is the origin of {{< math >}}$\{B\}${{< /math >}} w.r.t. {{< math >}}$\{A\}${{< /math >}}.

We can use this to map between frames, {{< math >}}$^AP=~^AP_{B,\text{ORIG}}+~^BP${{< /math >}}. Therefore, {{< math >}}$^AP_{B,\text{ORIG}}${{< /math >}} is a translational mapping from {{< math >}}$\{B\}${{< /math >}} to {{< math >}}$\{A\}${{< /math >}}, and {{< math >}}$^AP${{< /math >}} and {{< math >}}$^BP${{< /math >}} are the positions of the particle in the respective frames.

### Rotation

Rotations are fundamentally more difficult in 3D than in 2D.

- There are three axes which can be rotated around
- These axes can either be fixed (static) or move with each rotation (non-static)
- The order of rotation, while philosophically unimportant, is of vital importance for consistency
- The rotation matrix formed by rotation in a particular order is not the same as the rotation matrix for a different order

![3D Rotation Example](3d_rotation.png)

Expressing {{< math >}}$\{B\}${{< /math >}} w.r.t. {{< math >}}$\{A\}${{< /math >}} requires rotation around three axes in general. The order of rotation is important, there is not a single unique mapping between the two frames.

#### Intrinsic vs. Extrinsic Rotation

{{% callout note %}}
This is also referred to as non-static/relative vs. static rotation.
{{% /callout %}}

For **intrinsic/relative rotation**, rotation is done around intermediate reference frames, and the rotation matrices are post-multiplied; this means that the first rotation matrix in the expression is the first rotation in the sequence.

For **extrinsic/static rotation**, all rotation is done around a fixed reference frame, and rotation matrices are pre-multiplied; this means the first rotation matrix in the expression is the last rotation in the sequence.

![Intrinsic vs. Extrinsic Rotations](assets/img/tutorials/kinematics/transformations/3d_extrinsic_intrinsic_rotation_comparison.png)

#### Extrinsic Rotation

A common way to rotate in 3D is the “fixed X-Y-Z” way:

1. Rotate about the X-axis (fixed)
2. Rotate about the Y-axis (fixed)
3. Rotate about the Z-axis (fixed)

This yields the rotation matrix {{< math >}}$^A_BR_{XYZ}(\alpha, \beta,\gamma)=R_Z(\alpha)R_Y(\beta)R_X(\gamma)${{< /math >}}

The individual rotation matrices are

{{< math >}}

$$
\begin{aligned}
    R_Z(\alpha) &=
    \begin{bmatrix}
        \cos\alpha & -\sin\alpha & 0 \\
        \sin\alpha & \cos\alpha & 0 \\
        0 & 0 & 1
    \end{bmatrix}\\
    R_Y(\beta) &=
    \begin{bmatrix}
        \cos\beta & 0 & \sin\beta \\
        0 & 1 & 0 \\
        -\sin\beta & 0 & \cos\beta
    \end{bmatrix}\\
    R_X(\gamma) &=
    \begin{bmatrix}
        1 & 0 & 0 \\
        0 & \cos\gamma & -\sin\gamma \\
        0 & \sin\gamma & \cos\gamma
    \end{bmatrix}
\end{aligned}
$$

{{< /math >}}

When combined together, it looks something like this

![Extrinsic Rotation](assets/img/tutorials/kinematics/transformations/3d_extrinsic_rotation.png)

The combined rotation matrix is

{{< math >}}

$$
{}^A_B R_{XYZ}(\alpha, \beta, \gamma) =
\begin{bmatrix}
    \cos\alpha \cos\beta & \cos\alpha \sin\beta \sin\gamma - \sin\alpha \cos\gamma & \cos\alpha \sin\beta \cos\gamma + \sin\alpha \sin\gamma \\
    \sin\alpha \cos\beta & \sin\alpha \sin\beta \sin\gamma + \cos\alpha \cos\gamma & \sin\alpha \sin\beta \cos\gamma - \cos\alpha \sin\gamma \\
    -\sin\beta & \cos\beta \sin\gamma & \cos\beta \cos\gamma
\end{bmatrix}
$$

{{< /math >}}

#### Intrinsic Rotation

The general idea is to rotate successively into intermediate (relative) reference frames

1. Perform one rotation
2. This rotation establishes a new frame
3. Perform another rotation
4. This rotation establishes a new frame
5. Perform a final rotation

This can be denoted {{< math >}}$^A_BR=~^A_{B'}R~^{B'}_{B^{\prime\prime}}R~^{B^{\prime\prime}}_BR$${{< /math >}}

As before, the rotation matrix comprises of a product of rotation matrices and must be carried out in a specific order.

{{% callout note %}}
There are various different conventions, we will use {{< math >}}$Z$-$Y$-$X${{< /math >}} Euler Angles.
{{% /callout %}}

![Intrinsic Rotation](assets/img/tutorials/kinematics/transformations/3d_intrinsic_rotation.png)

Noting that we rotate about the intermediate axes {{< math >}}$Y'${{< /math >}} and {{< math >}}$X''${{< /math >}}

{{< math >}}

$$
^A_BR_{ZY'X''}(\psi, \theta,\phi)=R_Z(\psi)R_{Y'}(\theta)R_{X''}(\phi)
$$

{{< /math >}}

Where our rotation matrices are

{{< math >}}

$$
\begin{aligned}
  R_Z(\psi) &=
  \begin{bmatrix}
    \cos \psi & -\sin \psi & 0 \\
    \sin \psi & \cos \psi & 0 \\
    0 & 0 & 1
  \end{bmatrix}\\
  R_Y(\theta) &=
  \begin{bmatrix}
    \cos \theta & 0 & \sin \theta \\
    0 & 1 & 0 \\
    -\sin \theta & 0 & \cos \theta
  \end{bmatrix}\\
  R_{X''}(\phi) &=
  \begin{bmatrix}
    1 & 0 & 0 \\
    0 & \cos \phi & -\sin \phi \\
    0 & \sin \phi & \cos \phi
  \end{bmatrix}
\end{aligned}
$$

{{< /math >}}

The rotation matrices have the same form as in fixed axis rotation but the axes about which the rotation takes place are different.

This gives an overall rotation matrix of

{{< math >}}

$$
R_{Z'X''Y''}(\psi, \theta, \phi) = \begin{bmatrix}
\cos\psi\cos\theta & \cos\psi\sin\theta\sin\phi - \sin\psi\cos\phi & \cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi \\
\sin\psi\cos\theta & \sin\psi\sin\theta\sin\phi + \cos\psi\cos\phi & \sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix}
$$

{{< /math >}}

### Homogeneous Transformations

Similar to in 2D, homogeneous transformations provide a way of combining rotations and translations

As with 2D, given a position w.r.t. frame {{< math >}}$\{B\}${{< /math >}} the position w.r.t. frame {{< math >}}$\{A\}${{< /math >}} can be obtained via

{{< math >}}

$$
^AP=~^AP_{B,\text{ORIG}}+~^A_BR~~^BP
$$

{{< /math >}}

Where {{< math >}}$^AP_{B,\text{ORIG}}${{< /math >}} is a vector describing a translation of the origin of frame {{< math >}}$\{B\}${{< /math >}} in {{< math >}}$\{A\}${{< /math >}} coordinates. {{< math >}}$^A_BR${{< /math >}} is a rotation matrix describing a rotation of the {{< math >}}$\{B\}${{< /math >}} axes w.r.t. frame {{< math >}}$\{A\}${{< /math >}}

Homogeneous transformations in 3D are given by

{{< math >}}

$$
\begin{align*}
  ^AP&=\begin{bmatrix}^A_BR&^AP_{B,\text{ORIG}}\end{bmatrix}\begin{bmatrix}^BP\\1\end{bmatrix}\\
  \implies \begin{bmatrix}^AP\\1\end{bmatrix}&=\begin{bmatrix}^A_BR&^AP_{B,\text{ORIG}}\\0&1\end{bmatrix}\begin{bmatrix}^BP\\1\end{bmatrix}\\
  \implies ^A\tilde{P}&=~^A_BT~~^B\tilde{P}
\end{align*}
$$

{{< /math >}}

Where

{{< math >}}

$$
^A_BT=\begin{bmatrix}^A_BR&^AP_{B,\text{ORIG}}\\0&1\end{bmatrix}
$$

{{< /math >}}

This can be expanded to

{{< math >}}

$$
{}^A_BT =
\begin{bmatrix}
    \cos\alpha \cos\beta & \cos\alpha \sin\beta \sin\gamma - \sin\alpha \cos\gamma & \cos\alpha \sin\beta \cos\gamma + \sin\alpha \sin\gamma  & {}^BP_x\\
    \sin\alpha \cos\beta & \sin\alpha \sin\beta \sin\gamma + \cos\alpha \cos\gamma & \sin\alpha \sin\beta \cos\gamma - \cos\alpha \sin\gamma & {}^BP_y\\
    -\sin\beta & \cos\beta \sin\gamma & \cos\beta \cos\gamma & {}^BP_z \\
    0 & 0 & 0 & 1
\end{bmatrix}
$$

{{< /math >}}

Items to note about 3D homogeneous transformation matrices:

- 4x4 matrix describing rotation/translation
- 9 entries describing rotation; 3 describing translation
- The basis of most kinematics equations

Similar to 2D, we find that {{< math >}}$^A_BT=~^A_CT~~^C_BT$${{< /math >}}. This transfers directly to our robotics applications in that {{< math >}}$^0_nT=^0_1T\dots~^{n-1}\_nT${{< /math >}} (c.f. kinematics chain image at the start of this page).
