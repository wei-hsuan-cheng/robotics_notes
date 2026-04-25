# Manipulability-Based Singularity Avoidance and Its Gradient

This note explains why Yoshikawa’s **manipulability measure** <a href="#ty85">[1]</a> is a useful way to formulate **singularity avoidance problems**, and then summarizes a compact derivation of its ***gradient***. The manipulability measure, also called the manipulability index, indicates the **ease** with which the robot can move in different directions <a href="#mr17">[2]</a>.

The key point is that singularity is fundamentally a **Jacobian rank-loss** problem. Since the Jacobian depends on the robot configuration, a singularity metric can be written as a scalar function of the configuration. In this text the Jacobian is built via the Product of Exponentials (PoE) in the body/end-effector frame. It matches standard Lie-group identities and is equivalent to product-rule derivations written directly with exponentials.

## Why manipulability is a good singularity metric

Let a task Jacobian be
$$
J_{\text{task}}(q) \in \mathbb{R}^{m \times n}, \qquad m \le n,
$$
where $m$ is the selected task dimension and $n$ is the number of actuated joints used for that task.

A **kinematic singularity** occurs when $J_{\text{task}}$ loses row rank. At such a configuration, at least one task space direction becomes difficult or impossible to realize movement with finite joint motion. Equivalently:

- at least one singular value $\sigma_i(J_{\text{task}})$ collapses toward zero,
- the task velocity ellipsoid shrinks along one or more principal axes,
- the inverse kinematics / resolved-rate mapping becomes ill-conditioned.

This is exactly why manipulability is convenient:
$$
w(q) = \sqrt{\det\!\big(J_{\text{task}} J_{\text{task}}^T\big)}
     = \prod_{i=1}^{m} \sigma_i(J_{\text{task}}).
$$

So:

- $w(q) > 0$ when the selected task Jacobian has full row rank,
- $w(q) = 0$ exactly at a singular configuration for that selected task,
- small $w(q)$ means the robot is close to a singular or poorly conditioned configuration.

This turns the singularity avoidance problem into a scalar inequality problem: instead of reasoning directly about rank deficiency, we can keep the robot away from singularity by enforcing a lower bound on $w$.

## Notation

- Joint vector: $\theta = [\theta_1\;\ldots\;\theta_n]^T$.
- Body/end–effector Jacobian: $J(\theta) = [J_1\;\cdots\;J_n] \in\mathbb{R}^{6\times n}$ with columns $J_i \in \mathbb{R}^6$.
- Yoshikawa manipulability <a href="#ty85">[1]</a>: $\displaystyle w(\theta) = \sqrt{\det\big(J J^T\big)}$.
- Manipulability gradient: $J_w(\theta) := \partial w(\theta) / \partial \theta \in\mathbb{R}^{1\times n}$.
- Hat/vee maps: $\hat{S} = S^{\wedge} \in \mathfrak{se}(3)$ for $S = (S^{\wedge})^{\vee}\in\mathbb{R}^6$, where we use screw order $S=[v^T\,\omega^T]^T$.
- The commutator (or Lie bracket) <a href="#js05">[1]</a>: $[A,B]=AB-BA$.
- The adjoint representations <a href="#mr17">[2]</a><a href="#js05">[3]</a><a href="#rm94">[4]</a> for Lie group $\mathrm{Ad}(\cdot)$ and Lie algebra $\mathrm{ad}(\cdot)$: $\mathrm{Ad}_T \in \mathbb{R}^{6\times6},\; \mathrm{ad}_S \in \mathbb{R}^{6\times6}$ with our $S = [v^T\,\omega^T]^T$ ordering
  $$
  \mathrm{ad}_{S} = \begin{bmatrix} [\omega]_\times & [v]_\times \\
                                        0               & [\omega]_\times \end{bmatrix},\text{ s.t. }
  (S \times \eta)^{\wedge} := (\mathrm{ad}_{S} \eta)^{\wedge} = [\hat{S},\hat{\eta}]
  $$
  where "$\times$" is the *twist* cross product, an extension of cross product from $\mathbb{R}^3$ to $\mathbb{R}^6$ <a href="#rm94">[4]</a>.

## General gradient of Yoshikawa’s $w$

Manipulability quantifies **proximity** to singularity, but its **gradient** is what makes it **actionable**: it provides the local joint-space direction of increasing manipulability, so the avoidance policy can guide the robot away from singular configurations rather than merely detect them.

The goal is then to solve for the manipulability gradient

$$
\boxed{\;J_w(\theta) := \frac{\partial w(\theta)}{\partial \theta}\in\mathbb{R}^{1\times n},\text{ s.t. } \dot{w} = J_w(\theta)\;\dot{\theta}\in\mathbb{R}.\;}
$$

Let $A(\theta) = J J^T \in \mathbb{R}^{6\times6}$. Then

$$
\frac{\partial w}{\partial \theta_j}
  = \frac{w}{2}\;\mathrm{tr}\!\left(A^{-1} \frac{\partial A}{\partial \theta_j}\right),
\qquad
\frac{\partial A}{\partial \theta_j} = \frac{\partial J}{\partial \theta_j} J^T + J \left(\frac{\partial J}{\partial \theta_j}\right)^T.
$$

Using symmetry of $A^{-1}$ and cyclic trace, this simplifies to the numerically convenient form <a href="#gm02">[5]</a>

$$
\boxed{\;\frac{\partial w}{\partial \theta_j} = w\,\mathrm{Tr}\!\left(\frac{\partial J}{\partial \theta_j}\,J^{\dagger}\right)\in\mathbb{R}.\;}
$$
Here $J^{\dagger}$ denotes the Moore–Penrose pseudoinverse (for full row rank, $J^{\dagger}=J^T (J J^T)^{-1}$; near singularities use a damped form).

## PoE body Jacobian and its derivatives

For PoE in the body frame, the $i$‑th column of $J(\theta)$ is <a href="#mr17">[2]</a>

$$
J_i(\theta) = \mathrm{Ad}_{\,e^{-\hat{S}_n\theta_n}\cdots e^{-\hat{S}_{i+1}\theta_{i+1}}} \, S_i.
$$

First define $M_i := e^{-\hat{S}_n\theta_n}\cdots e^{-\hat{S}_{i+1}\theta_{i+1}}$, so that $J_i = \mathrm{Ad}_{M_i} S_i \Leftrightarrow \hat{J}_i = M_i\,\hat{S}_i\,M_i^{-1}$. 

- Differentiating $\hat{J}_i(\theta)$ w.r.t. $\theta_j$ yields two regimes:

  1) If $j \le i$, $\, e^{-\hat{S}_j\theta_j}$ is not in $M_i$, so $\partial J_i/\partial\theta_j = 0$.

  2) If $j>i$,
  $$
  \frac{\partial \hat{J}_i}{\partial \theta_j}
  = \underbrace{\left(\frac{\partial M_i}{\partial\theta_j}\right) \hat{S}_i M_i^{-1} + M_i\hat{S}_i\left(\frac{\partial M_i^{-1}}{\partial\theta_j}\right)}_{\text{product rule}}
  = \big[(\partial M_i/\partial\theta_j)M_i^{-1},\; \hat{J}_i\big].
  $$

  <details>
  <summary>Derivation: product‑rule terms become a commutator</summary>

  For any differentiable, invertible matrix $M(\theta)$ and constant matrix $X$,
  $$
  \frac{\partial}{\partial\theta}\big(M X M^{-1}\big)
  = (\partial M) X M^{-1} + M X (\partial M^{-1}).
  $$
  Use the inverse derivative identity, obtained from $I = M M^{-1}$:
  $$
  0 = (\partial M)M^{-1} + M(\partial M^{-1})\;\;\Rightarrow\;\; \partial M^{-1} = -\,M^{-1}(\partial M)M^{-1}.
  $$
  Substitute and define $Q := (\partial M)M^{-1}$:
  $$
  \begin{aligned}
  \frac{\partial}{\partial\theta}\big(M X M^{-1}\big)
  &= (\partial M) X M^{-1} - M X M^{-1} (\partial M) M^{-1}\\
  &= Q\,(M X M^{-1}) - (M X M^{-1})\,Q\\
  &= \big[\,Q,\; M X M^{-1}\,\big].
  \end{aligned}
  $$
  Applying this with $M=M_i$, $X=\hat{S}_i$ and $M X M^{-1} = \hat{J}_i$ gives
  $$
  \frac{\partial \hat{J}_i}{\partial \theta_j} = \big[\,(\partial M_i/\partial\theta_j) M_i^{-1},\; \hat{J}_i\,\big].
  $$

  </details>

- Now factor $M_i = F\,e^{-\hat{S}_j\theta_j}G$ with $F=e^{-\hat{S}_n\theta_n}\cdots e^{-\hat{S}_{j+1}\theta_{j+1}}$ and $G=e^{-\hat{S}_{j-1}\theta_{j-1}}\cdots e^{-\hat{S}_{i+1}\theta_{i+1}}$. Then
  $$
  (\partial M_i/\partial\theta_j)M_i^{-1} = F(-\hat{S}_j)F^{-1} = -\,(\mathrm{Ad}_F S_j)^{\wedge} = -\,\hat{J}_j.
  $$

  <details>
  <summary>Details</summary>

  1. Only the $j$‑th exponential depends on $\theta_j$, so with $E_j:=e^{-\hat{S}_j\theta_j}$ and constants (w.r.t. $\theta_j$) $F, G$,
    $$M_i = F\,E_j\,G,\qquad \frac{\partial M_i}{\partial\theta_j} = F\,\frac{\partial E_j}{\partial\theta_j}\,G.$$
  2. For a constant matrix $A$, $\frac{\partial}{\partial\theta}e^{A\theta}=A e^{A\theta}$. Hence $\frac{\partial E_j}{\partial\theta_j}=(-\hat{S}_j)E_j$ and
    $$\frac{\partial M_i}{\partial\theta_j} = F\,(-\hat{S}_j)\,E_j\,G.$$
  3. Right‑multiply by $M_i^{-1}=G^{-1}E_j^{-1}F^{-1}$ and cancel:
    $$\begin{aligned}
    (\partial M_i/\partial\theta_j)M_i^{-1}
    &= F(-\hat{S}_j)E_j\,G\;G^{-1}E_j^{-1}F^{-1}\\
    &= F(-\hat{S}_j)\,\underbrace{E_jE_j^{-1}}_{I}\,F^{-1} = F(-\hat{S}_j)F^{-1}.
    \end{aligned}$$
  4. Use the adjoint identity $(\mathrm{Ad}_T S)^{\wedge}=T\hat{S}T^{-1}$. With $T=F$,
    $$F(-\hat{S}_j)F^{-1}=-(\mathrm{Ad}_F S_j)^{\wedge}.$$
  5. For the body Jacobian, $J_j(\theta)=\mathrm{Ad}_F S_j$, so $(\mathrm{Ad}_F S_j)^{\wedge}=\hat{J}_j$ and the chain closes.

  </details>

Therefore,
$$
\boxed{\;\frac{\partial \hat{J}_i}{\partial \theta_j} = -\,[\hat{J}_j,\hat{J}_i] \;\;\Leftrightarrow\;\; \frac{\partial J_i}{\partial \theta_j} = -\,\mathrm{ad}_{J_j}\,J_i\quad \forall j>i;\qquad \frac{\partial J_i}{\partial \theta_j}=0\quad \forall j\le i.\;}
$$

>These are the standard “triangular” derivative identities for the body Jacobian. For the space Jacobian the signs/inequalities swap: $\partial J^s_i/\partial\theta_j = \mathrm{ad}_{J^s_j} J^s_i$ for $j<i$, zero otherwise.

## Assembling the manipulability gradient

Let the manipulator Hessian be the third‑order tensor $\mathcal{H} \in \mathbb{R}^{6\times n\times n}$ with components
$$
\mathcal{H}_{k,i,j} := \frac{\partial (J)_{k,i}}{\partial \theta_j},\quad k\in\{1,\dots,6\},\; i,j\in\{1,\dots,n\}.
$$
Denote the $j$‑th slice along the third index by $H^{(j)} := \partial J/\partial\theta_j \in \mathbb{R}^{6\times n}$. From the body identities above,
$$
H^{(j)} = \big[\,-\,\mathrm{ad}_{J_j}J_1\;\cdots\;-\,\mathrm{ad}_{J_j}J_{j-1}\;\;0\;\cdots\;0\big].
$$

The gradient $J_w\in\mathbb{R}^{1\times n}$ thus has its $j$-th component,
$$
\boxed{\;J_{w,j} = \frac{\partial w}{\partial \theta_j} = w\,\mathrm{Tr}(H^{(j)} \, J^{\dagger})\in \mathbb{R}\;}
$$
and using the triangular structure,
$$
\mathrm{Tr}(H^{(j)} J^{\dagger}) = -\sum_{i=1}^{j-1} (J^{\dagger})_{i,:}\,\big(\mathrm{ad}_{J_j}J_i\big).
$$
Here $(J^{\dagger})_{i,:}\in\mathbb{R}^{1\times 6}$ is row $i$ of $J^{\dagger}$.

## Practical notes

- Near singularities, $(J J^T)^{-1}$ may be ill‑conditioned; use a damped inverse $(J J^T + \lambda^2 I)^{-1}$ in practice.
- In implementation it is often numerically cleaner to compute $\displaystyle w = \prod_i \sigma_i(J_{\text{task}})$ directly from an SVD rather than forming $\det(JJ^T)$ explicitly.
- Dimensions: $J\in \mathbb{R}^{6\times n}$, $J^{\dagger}\in \mathbb{R}^{n\times 6}$, $\mathcal{H}\in \mathbb{R}^{6\times n\times n}$ with slice $H^{(k)}\in \mathbb{R}^{6\times n}$; $\mathrm{Tr}(H^{(k)} J^{\dagger})$ is scalar.
- The body identities imply $\partial J_i/\partial \theta_i = 0$. For the space Jacobian, self‑derivatives are also zero but the triangular structure flips.

## References

- <a name="ty85">[1]</a> T. Yoshikawa, “Manipulability of Robotic Mechanisms,” IJRR, 1985.
- <a name="mr17">[2]</a> K. M. Lynch, F. C. Park, Modern Robotics: Mechanics, Planning, and Control, 2017.
- <a name="js05">[3]</a> J. Selig, Geometric Fundamentals of Robotics, 2005.
- <a name="rm94">[4]</a> R. M. Murray, Z. Li, S. S. Sastry, A Mathematical Introduction to Robotic Manipulation, 1994.
- <a name="gm02">[5]</a> G. Marani, J. Kim, J. Yuh, W. K. Chung, “A Real‑Time Approach for Singularity Avoidance in Resolved Motion Rate Control of Robotic Manipulators,” ICRA 2002.
