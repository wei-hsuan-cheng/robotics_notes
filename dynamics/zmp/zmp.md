# Zero Moment Point (ZMP) in Legged Robotics

The **Zero Moment Point (ZMP)** is a key concept in legged robotics, especially for humanoid robots, to ensure stable gait and balancing. It refers to the point on the ground where the **net moment** from the robot’s forces (gravity, ground reaction, etc.) has no horizontal component. For a robot to remain stable, its **ZMP** must stay within the **support polygon**, which is the area defined by the contact points of its feet with the ground.

This explanation provides an intuitive understanding of ZMP, its mathematical derivation, usages, limitations, and a comparison between the simple **lumped-mass model** and the more complex **centroidal dynamics** used in advanced robotics. Additionally, the simplified form of the **Linear Inverted Pendulum Model (LIPM)** is included.

---

## 1. **Intuition Behind ZMP**

The **Zero Moment Point (ZMP)** is the point on the ground where the resultant contact forces and moments produced by the robot do **not generate a tipping moment**. It is crucial to ensure that the robot remains stable and does not tip over.

### Key Insights:
- **ZMP Definition**: The point on the ground where the total moment acting on the robot (due to external forces like gravity and ground reaction forces) equals zero in the horizontal plane (x and y directions).
- For a robot to be stable, the ZMP must remain **inside** the **support polygon**. If the ZMP moves outside this area, the robot will fall.

---

## 2. **Mathematical Derivation of ZMP**

The ZMP can be derived using the Newton-Euler equations for the robot's motion. Here, we’ll consider the robot as a collection of point masses \(m_i\) located at different positions \(r_i = (x_i, y_i, z_i)\).

### 2.1 **Newton-Euler Equations**

For each mass element \(i\) of the robot, the Newton-Euler equation is:

\[
m_i \ddot{r}_i = f_i^{\text{ext}} + m_i \vec{g}
\]

Where:
- \(m_i\): mass of the \(i\)-th link
- \(\ddot{r}_i = (\ddot{x}_i, \ddot{y}_i, \ddot{z}_i)\): acceleration of the \(i\)-th link
- \(f_i^{\text{ext}}\): external forces acting on the \(i\)-th link (including ground reaction forces)
- \(\vec{g} = (0, 0, -g)\): gravitational acceleration vector

Summing over all the masses:

\[
F_x = \sum_i m_i \ddot{x}_i,\quad
F_y = \sum_i m_i \ddot{y}_i,\quad
F_z = \sum_i m_i (\ddot{z}_i + g)
\]

Where:
- \(F_x, F_y, F_z\): total external forces along the x, y, and z axes

---

### 2.2 **Moment Calculation**

Next, we calculate the moment of the external forces about the origin on the ground plane \(z=0\). The external moment is:

\[
M_O = \sum_i r_i \times m_i(\ddot{r}_i - g_{vec})
\]

Where the cross product is computed as:

\[
r_i = (x_i, y_i, z_i),\quad m_i(\ddot{r}_i - g) = (m_i \ddot{x}_i, m_i \ddot{y}_i, m_i (\ddot{z}_i + g))
\]

This gives the moments:

\[
M_x = \sum_i \left[ y_i m_i (\ddot{z}_i + g) - z_i m_i \ddot{y}_i \right]
\]

\[
M_y = \sum_i \left[ z_i m_i \ddot{x}_i - x_i m_i (\ddot{z}_i + g) \right]
\]

\[
M_z = \sum_i \left[ x_i m_i \ddot{y}_i - y_i m_i \ddot{x}_i \right]
\]

---

### 2.3 **ZMP Definition**

The ZMP is the point where the moment due to the ground reaction force is zero. It is given by:

\[
M_O = p_Z \times F + M_Z
\]

Where:
- \(p_Z = (x_Z, y_Z, 0)\) is the ZMP
- \(F = (F_x, F_y, F_z)\) is the resultant force at the ZMP
- \(M_Z\) is any residual moment at the ZMP (which is zero in the horizontal plane)

Therefore, for the horizontal moments:

\[
M_x = y_Z F_z,\quad M_y = -x_Z F_z
\]

Thus, the ZMP coordinates are:

\[
x_Z = -\frac{M_y}{F_z},\quad y_Z = \frac{M_x}{F_z}
\]

---

### 2.4 **Final Closed-Form Equation**

By substituting the expressions for \(M_x\) and \(M_y\), we get the closed-form expressions for the ZMP coordinates:

\[
x_{zmp} = \frac{\sum_i m_i (\ddot{z}_i + g) x_i - \sum_i m_i \ddot{x}_i z_i}{\sum_i m_i (\ddot{z}_i + g)}
\]

\[
y_{zmp} = \frac{\sum_i m_i (\ddot{z}_i + g) y_i - \sum_i m_i \ddot{y}_i z_i}{\sum_i m_i (\ddot{z}_i + g)}
\]

These equations represent the ZMP location in terms of the robot's mass distribution, accelerations, and gravitational forces.

---

## 3. **Usage of ZMP in Robotics**

### 3.1 **Applications in Gait and Balancing**

ZMP is a fundamental concept used in **gait planning** and **balancing** in legged robots. It is typically used in the following:

1. **Walking Gait Planning**: The ZMP trajectory is planned such that it stays inside the support polygon during double and single support phases of walking.
   
2. **Balancing**: During standing, ZMP must remain inside the support polygon. If the ZMP shifts outside, corrective actions (such as shifting the body or taking a step) are required to maintain stability.

3. **Push Recovery**: In case the robot is pushed, the ZMP is monitored to decide if corrective action (like stepping) is required to prevent a fall.

### 3.2 **Controllers and Algorithms**

- **Preview Control**: ZMP is used in preview control (e.g., Kajita's method) to predict and adjust the robot's motion.
- **Model Predictive Control (MPC)**: ZMP is incorporated as a constraint in walking controllers like **OCS2 MPC** to generate stable walking gaits.

---

## 4. **Centroidal Dynamics vs Lumped-Mass Model**

While the **lumped-mass model** provides a simple approximation of ZMP by treating the robot’s center of mass (COM) as a single point mass, **centroidal dynamics** introduces a more complex model that accounts for rotational inertia and angular momentum effects. This results in a more accurate and general formulation of ZMP, especially useful for dynamic motions.

### 4.1 **Lumped-Mass Model** (Simplified)

In the **lumped-mass model**, the robot is treated as a point mass located at the **COM**, and the ZMP is determined based on the horizontal acceleration and position of the COM:

\[
x_Z = x_C - \frac{z_C}{g} \ddot{x}_C,\quad y_Z = y_C - \frac{z_C}{g} \ddot{y}_C
\]

Where:
- \(x_C, y_C\): COM position
- \(z_C\): COM height
- \(\ddot{x}_C, \ddot{y}_C\): accelerations of COM in x and y directions
- \(g\): gravitational acceleration

This approach simplifies the robot’s dynamics, treating it as a single point mass and ignoring its rotational inertia and angular momentum.

### 4.2 **Centroidal Dynamics (More Complex)**

In contrast, **centroidal dynamics** involves modeling the robot’s **entire mass distribution**, including **rotational inertia** and **angular momentum**. In this framework, the ZMP is determined not only by the COM motion but also by the **distribution of mass** and the **robot’s rotational behavior**. The effect of **angular momentum** and **torques** about the COM must be accounted for, making the ZMP formulation more complex and robust for dynamic motions.

For instance, in centroidal dynamics, the ZMP calculation includes contributions from the robot's **momentum** and **rotational inertia**, which modify the horizontal forces and moments. The more advanced centroidal control techniques take these into account, making the robot more stable during fast, dynamic movements like running, jumping, or quickly shifting its body.

### 4.3 **ZMP in Centroidal Dynamics**

In centroidal dynamics, the ZMP is derived from the **centroidal angular momentum** and **contact forces**, yielding a more accurate representation of the robot's overall motion and stability. The ZMP equation incorporates not only the forces from COM motion but also contributions from the robot’s **joint torques** and **angular momentum**.

This results in a more **generalized** ZMP formula that includes:

- **Angular momentum**: The robot’s rotational inertia and the effect of torque on its stability.
- **Mass distribution**: The full distribution of mass across the robot’s body, not just the COM.

The resulting ZMP form is more **complex** than the lumped-mass case, but is necessary for higher-performance robots operating in dynamic environments.

---

## 5. **LIPM Simplified Form for ZMP**

The **Linear Inverted Pendulum Model (LIPM)** is widely used in humanoid robotics for stable walking. It simplifies the ZMP calculation by approximating the robot's motion as a point mass above a point of support. The simplified ZMP formula for a **single point mass** moving in the \(x\) and \(y\) directions is given by:

\[
x_{zmp} = x_{com} - \frac{z_{com}}{g} \ddot{x}_{com}, \quad y_{zmp} = y_{com} - \frac{z_{com}}{g} \ddot{y}_{com}
\]

Where:
- \(x_{com}, y_{com}\): position of the center of mass (COM)
- \(z_{com}\): height of the COM
- \(\ddot{x}_{com}, \ddot{y}_{com}\): accelerations of the COM in the \(x\) and \(y\) directions
- \(g\): gravitational acceleration

This form of ZMP is valid for **quasi-static motions** (slow walking or standing) and is widely used for **gait planning** in humanoid robots.

---

## 6. **Pros, Caveats, and Limitations of ZMP**

### 6.1 **Pros**
- **Simple Concept**: ZMP is easy to understand and provides a clear criterion for stability.
- **Useful for Slow, Quasi-Static Motion**: It works well for stable, slow walking or standing robots.
- **Wide Adoption**: ZMP has been used in many humanoid robots like ASIMO and HRP series.

### 6.2 **Caveats**
- **Does Not Handle Dynamic Motion Well**: ZMP is less effective for dynamic gaits (e.g., running, jumping) where the robot's motion involves significant inertia and angular momentum.
- **Assumes a Flat Ground**: ZMP calculations assume a flat ground surface, making it less suitable for rough terrain or non-level surfaces.

### 6.3 **Limitations**
- **Angular Momentum**: ZMP does not account for angular momentum well, especially in highly dynamic movements.
- **Reduced Accuracy for Large Robots**: For larger robots, the mass distribution and joint dynamics can complicate ZMP control, requiring more sophisticated models.
- **No Tipping Consideration**: ZMP only considers the horizontal moment, not the tipping dynamics, making it less reliable when large disturbances occur.

---

## 7. **Summary**

The **Zero Moment Point (ZMP)** is a critical concept in legged robotics that ensures stable walking and balancing. It defines a point on the ground where the net moment from all external forces equals zero. ZMP is primarily used for:

1. **Balancing**: Ensuring the ZMP stays within the support polygon.
2. **Gait Planning**: Generating stable walking patterns by maintaining the ZMP trajectory within support areas.
3. **Push Recovery**: Detecting when corrective actions (like stepping) are needed to prevent tipping.

While ZMP is highly useful for quasi-static motion, it has limitations in dynamic environments, where more advanced methods (like **Centroidal Dynamics** and **Capture Point**) are used. Understanding ZMP and its limitations is crucial for designing stable legged robots, especially for applications involving walking and balancing.