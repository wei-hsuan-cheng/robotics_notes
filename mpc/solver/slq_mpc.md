# Sequential Linear Quadratic Model Predictive Control (SLQ-MPC)

- Reference literature for SLQ-MPC
    - <a name="icra16">[1]</a> [(ICRA2016) Fast nonlinear Model Predictive Control for unified trajectory optimization and tracking](https://ieeexplore.ieee.org/document/7487274)
    - <a name="icra17">[2]</a> [(ICRA2017) An Efficient Optimal Planning and Control Framework For Quadrupedal Locomotion](https://ieeexplore.ieee.org/document/7989016)
---

## 1. Nonlinear optimal control problem (discrete time)

We start with a **discrete-time** nonlinear system
$$
    x_{k+1} = f(x_k, u_k), \qquad k = 0,\dots,N-1,
$$

and a finite-horizon cost

$$
    \mathcal{J}
    = h(x_N) + \sum_{k=0}^{N-1} \ell(x_k, u_k).
$$

Goal:  
Find a control sequence $u_0,\dots,u_{N-1}$ that **minimizes** $\mathcal{J}$ subject to the dynamics.

SLQ/iLQR solves this **iteratively** by locally approximating the problem as LQ and using dynamic programming (Bellman).

---

## 2. Nominal trajectory and deviation variables

At iteration $i$, we have a **nominal trajectory**
$$
    \{x_k^n, u_k^n\}_{k=0}^{N-1}, \qquad x_0^n \text{ given}.
$$

We work in **deviation coordinates** around this trajectory:
$$
    \delta x_k := x_k - x_k^n,
    \qquad
    \delta u_k := u_k - u_k^n.
$$

We will compute increments $\delta u_k$ (and thus update $u_k^{\text{new}} = u_k^n + \delta u_k$) to reduce the cost.

---

## 3. Linearization of the dynamics

Linearize the nonlinear dynamics around $(x_k^n, u_k^n)$:
$$
    f(x_k, u_k) \approx f(x_k^n, u_k^n) 
    + A_k (x_k - x_k^n) 
    + B_k (u_k - u_k^n),
$$

with Jacobians
$$
    A_k := \left.\frac{\partial f}{\partial x}\right|_{(x_k^n, u_k^n)},
    \qquad
    B_k := \left.\frac{\partial f}{\partial u}\right|_{(x_k^n, u_k^n)}.
$$

Because the nominal satisfies
$$
    x_{k+1}^n = f(x_k^n, u_k^n),
$$

the constant term cancels, and we obtain the **linearized deviation dynamics**

$$
    \boxed{
    \delta x_{k+1} = A_k \delta x_k + B_k \delta u_k.
    }
$$

This is a linear time-varying (LTV) system.

---

## 4. Quadratic expansion of the cost

### 4.1 Stage cost expansion

Second-order Taylor expansion of the stage cost $\ell(x_k,u_k)$ at $(x_k^n,u_k^n)$:
$$
    \begin{aligned}
    \ell(x_k,u_k)
    &\approx \ell_k^n
    + \ell_{x,k}^\top \delta x_k
    + \ell_{u,k}^\top \delta u_k \\
    &\quad
    + \tfrac12\,\delta x_k^\top L_{xx,k}\,\delta x_k
    + \delta x_k^\top L_{xu,k}\,\delta u_k
    + \tfrac12\,\delta u_k^\top L_{uu,k}\,\delta u_k.
    \end{aligned}
$$

SLQ (as used in the paper) uses a **Gauss–Newton style approximation** and typically:

- for common tracking costs there is **no explicit $x$-$u$ cross term**, so $L_{xu,k}=0$ exactly, or  
- simply **drops** $L_{xu,k}$ to keep the structure LQR-like.

So we approximate
$$
L_{xu,k} \approx 0,
$$

giving
$$
    \ell(x_k,u_k)
    \approx
    \ell_k^n
    + \ell_{x,k}^\top \delta x_k
    + \ell_{u,k}^\top \delta u_k
    + \tfrac12\,\delta x_k^\top L_{xx,k}\,\delta x_k
    + \tfrac12\,\delta u_k^\top L_{uu,k}\,\delta u_k.
$$

Define the **LQ coefficients**:
$$
    Q_k := L_{xx,k},\quad
    R_k := L_{uu,k},\quad
    q_k := \ell_{x,k},\quad
    r_k := \ell_{u,k}.
$$

Then
$$
    \boxed{
    \ell(x_k,u_k) \approx
    \ell_k^n
    + q_k^\top \delta x_k
    + r_k^\top \delta u_k
    + \tfrac12\,\delta x_k^\top Q_k \delta x_k
    + \tfrac12\,\delta u_k^\top R_k \delta u_k.
    }
$$


### 4.2 Terminal cost expansion

Similarly, expand the terminal cost $h(x_N)$ at $x_N^n$:
$$
    h(x_N) \approx h_N^n
    + h_{x,N}^\top \delta x_N
    + \tfrac12\,\delta x_N^\top H_N \delta x_N,
$$

and define
$$
    P_N := H_N, \qquad p_N := h_{x,N}.
$$

These are the **boundary conditions** for the value-function recursion.

---

## 5. Value function and Bellman equation

### 5.1 Original value function

For the discrete-time problem, the **value function** at time $k$ is
$$
V_k(x) := \min_{u_k,\dots,u_{N-1}}
\left[
h(x_N) + \sum_{i=k}^{N-1} \ell(x_i,u_i)
\right],
$$
subject to the dynamics starting from $x_k = x$.

The **value function** tells: 
> If at time $𝑘$ I am in state $x$, what is the *minimum possible future cost* I can achieve, assuming I *act optimally* from now on?”

The **Bellman equation** (principle of optimality) is
$$
V_k(x) = \min_{u}
\big[\,\ell(x,u) + V_{k+1}(f(x,u))\,\big].
$$
At the final time $N$:
$$
V_N(x) = h(x).
$$

This **principle of optimality** is crucial:
> *Whatever the optimal strategy from now to the end is, its first action plus the optimal strategy afterwards must jointly be optimal.*  
> Mathematically, this is encoded in the Bellman equation.


### 5.2 Local quadratic approximation of the value function

We approximate $V_k$ **locally** around the nominal state $x_k^n$.  
Define $\delta x_k := x_k - x_k^n$ and write a second-order Taylor expansion:
$$
    V_k(x_k^n + \delta x_k)
    \approx
    s_k + p_k^\top \delta x_k
    + \tfrac12\,\delta x_k^\top P_k \delta x_k,
$$

where we define
$$
    s_k := V_k(x_k^n),
    \quad
    p_k := \left.\frac{\partial V_k}{\partial x}\right|_{x_k^n},
    \quad
    P_k := \left.\frac{\partial^2 V_k}{\partial x^2}\right|_{x_k^n}.
$$

For brevity, we denote this as
$$
    \boxed{
    V_k(\delta x_k) \approx
    s_k + p_k^\top \delta x_k
    + \tfrac12\,\delta x_k^\top P_k \delta x_k,
    }
$$

where $V_k(\delta x_k) := V_k(x_k^n + \delta x_k)$ for notation simplicity.

---

## 6. Q-function (cost-to-go for one step) and Bellman minimization

To avoid confusion with the matrix $Q_k$, we denote the **dynamic programming Q-function** by:
$$
\mathcal{Q}_k(\delta x_k,\delta u_k)
:= \ell(x_k,u_k) + V_{k+1}(\delta x_{k+1}),
$$
with
$$
\delta x_{k+1} = A_k \delta x_k + B_k \delta u_k.
$$

From the Bellman equation:
$$
V_k(\delta x_k) = \min_{\delta u_k} \mathcal{Q}_k(\delta x_k,\delta u_k).
$$

### 6.1 Substitute expansions

Stage cost:
$$
    \ell(x_k,u_k) \approx
    \ell_k^n
    + q_k^\top \delta x_k
    + r_k^\top \delta u_k
    + \tfrac12\,\delta x_k^\top Q_k \delta x_k
    + \tfrac12\,\delta u_k^\top R_k \delta u_k.
$$

Next value function (using linearized dynamics):
$$
    \begin{aligned}
    V_{k+1}(\delta x_{k+1})
    &\approx s_{k+1} + p_{k+1}^\top \delta x_{k+1}
    + \tfrac12\,\delta x_{k+1}^\top P_{k+1}\delta x_{k+1} \\[0.3em]
    &= s_{k+1}
    + p_{k+1}^\top (A_k\delta x_k + B_k\delta u_k) \\
    &\quad + \tfrac12 (A_k\delta x_k + B_k\delta u_k)^\top
    P_{k+1}(A_k\delta x_k + B_k\delta u_k).
    \end{aligned}
$$

Expand the quadratic term:
$$
    \begin{aligned}
    &\tfrac12 (A_k\delta x_k + B_k\delta u_k)^\top
    P_{k+1}(A_k\delta x_k + B_k\delta u_k) \\
    &= \tfrac12\,\delta x_k^\top A_k^\top P_{k+1}A_k \delta x_k
    + \tfrac12\,\delta u_k^\top B_k^\top P_{k+1}B_k \delta u_k \\
    &\quad + \delta u_k^\top B_k^\top P_{k+1}A_k \delta x_k.
    \end{aligned}
$$

Putting everything into $\mathcal{Q}_k$:
$$
\begin{aligned}
\mathcal{Q}_k(\delta x_k,\delta u_k)
&= \ell(x_k,u_k) + V_{k+1}(\delta x_{k+1}) \\
&\approx \text{const.} \\
&\quad + \delta x_k^\top \big(q_k + A_k^\top p_{k+1}\big) \\
&\quad + \delta u_k^\top \big(r_k + B_k^\top p_{k+1}\big) \\
&\quad + \tfrac12\,\delta x_k^\top \big(Q_k + A_k^\top P_{k+1}A_k\big)\delta x_k \\
&\quad + \tfrac12\,\delta u_k^\top \big(R_k + B_k^\top P_{k+1}B_k\big)\delta u_k \\
&\quad + \delta u_k^\top \big(B_k^\top P_{k+1}A_k\big)\delta x_k.
\end{aligned}
$$

Define:
$$
    \begin{aligned}
    H_k &:= R_k + B_k^\top P_{k+1}B_k, \\
    G_k &:= B_k^\top P_{k+1}A_k, \\
    g_k &:= r_k + B_k^\top p_{k+1}, \\
    \tilde{Q}_k &:= Q_k + A_k^\top P_{k+1}A_k, \\
    \tilde{q}_k &:= q_k + A_k^\top p_{k+1}.
    \end{aligned}
$$

Then the Q-function can be written as
$$
    \boxed{
    \begin{aligned}
    \mathcal{Q}_k(\delta x_k,\delta u_k)
    &= \text{const.}
    + \tfrac12\,\delta u_k^\top H_k\,\delta u_k
    + \delta u_k^\top (G_k\delta x_k + g_k) \\
    &\quad + \tfrac12\,\delta x_k^\top \tilde{Q}_k \delta x_k
    + \delta x_k^\top \tilde{q}_k.
    \end{aligned}
    }
$$

---

## 7. Principle of optimality: minimize the quadratic in $\delta u_k$

By the **Bellman equation**,
$$
    V_k(\delta x_k) = \min_{\delta u_k} \mathcal{Q}_k(\delta x_k,\delta u_k).
$$

We have a **quadratic function in $\delta u_k$**:
$$
    \mathcal{Q}_k(\delta x_k,\delta u_k)
    = \tfrac12\,\delta u_k^\top H_k\delta u_k
    + \delta u_k^\top (G_k\delta x_k + g_k)
    + (\text{terms independent of }\delta u_k).
$$

The **principle of optimality** says:  
> The optimal control at time $k$ *minimizes the cost-to-go from that point onward*.

So we set the gradient w.r.t. $\delta u_k$ to zero:
$$
    \frac{\partial \mathcal{Q}_k}{\partial \delta u_k}
    = H_k\delta u_k + G_k\delta x_k + g_k = 0.
$$

Solve for $\delta u_k$:
$$
    \boxed{
    \delta u_k^\star
    = l_k + K_k \delta x_k,
    \quad
    K_k := -H_k^{-1}G_k,
    \quad
    l_k := -H_k^{-1}g_k.
    }
$$

- $K_k$ is the **feedback gain** (same form as LQR).  
- $l_k$ is the **feedforward increment** (appears because we have nonzero gradients $r_k$ and $p_{k+1}$, *i.e.*, tracking/nonzero nominal).

This is the **optimal affine control law** for the local LQ subproblem at time $k$.

---

## 8. Backward recursion for the value function coefficients

We now plug the optimal law $\delta u_k^\star$ back into $\mathcal{Q}_k$ and identify the resulting expression as the new value function:
$$
    V_k(\delta x_k) = \min_{\delta u_k} \mathcal{Q}_k(\delta x_k,\delta u_k)
    = \mathcal{Q}_k(\delta x_k,\delta u_k^\star).
$$

We know that we want
$$
    V_k(\delta x_k)
    \approx s_k + p_k^\top \delta x_k
    + \tfrac12\,\delta x_k^\top P_k \delta x_k.
$$

By expanding $\mathcal{Q}_k(\delta x_k,\delta u_k^\star)$ and matching coefficients of $\delta x_k$ and $\delta x_k \delta x_k^\top$, we obtain the **Riccati-like backward recursions**:
$$
    \boxed{
    \begin{aligned}
    P_k
    &= \tilde{Q}_k
    + K_k^\top H_k K_k
    + K_k^\top G_k
    + G_k^\top K_k, \\[0.3em]
    p_k
    &= \tilde{q}_k
    + K_k^\top H_k l_k
    + G_k^\top l_k
    + K_k^\top g_k.
    \end{aligned}
    }
$$

(Scalar offsets $s_k$ also update but do not affect the policy, so often omitted.)

Together with the boundary conditions $P_N = H_N,\, p_N = h_{x,N}$, this defines a **backward pass**:

1. Start from $k=N$: known $P_N, p_N$.  
2. For $k=N-1,\dots,0$:
   - Compute $H_k, G_k, g_k$.  
   - Compute $K_k, l_k$.  
   - Update $P_k, p_k$.

This is the SLQ/iLQR **Riccati backward sweep**.

---

## 9. Forward rollout and line search (one SLQ iteration)

One **SLQ (iLQR) iteration** consists of:

1. **Forward rollout (nonlinear)**  
   Given current control law (nominal) $u_k^n$, simulate the **nonlinear** dynamics
   $$
    x_{k+1}^n = f(x_k^n, u_k^n)
   $$
   from the current initial state to get the nominal trajectory $\{x_k^n, u_k^n\}$.

2. **Backward pass (LQ subproblem)**  
   - Linearize dynamics: $A_k,B_k$.  
   - Quadratize cost: $Q_k,R_k,q_k,r_k$.  
   - Run the Riccati-like backward recursion to compute $K_k, l_k$.

3. **Policy update via line search**  
   Using the affine law $\delta u_k = l_k + K_k\delta x_k$, define the **candidate control law** for a step size $\alpha \in (0,1]$:
   $$
    u_k^{\text{cand}}(\alpha)
    = u_k^n + \alpha\, l_k + K_k\big(x_k^{\text{cand}}(\alpha) - x_k^n\big).
   $$
   Here $\alpha$ scales only the feedforward increment $l_k$, while $K_k$ is kept fixed and is used during the rollout to stabilize deviations.

   Roll out the nonlinear dynamics with this candidate law, compute the new cost $\mathcal{J}_{\text{cand}}(\alpha)$, and use a **line search** on $\alpha$ until the cost decreases sufficiently.

4. Set $\{x_k^n, u_k^n\}$ to the new trajectory and repeat until convergence (or max iterations).

At convergence you obtain a locally optimal **time-varying affine feedback policy**:
$$
    u_k(x_k) = u_k^n + l_k + K_k(x_k - x_k^n).
$$

This is iLQR/SLQ as a solver for the **single-shot** finite-horizon nonlinear OCP.

---

## 10. SLQ-MPC: using iLQR in a receding-horizon loop

SLQ-MPC wraps the SLQ solver inside an **MPC loop**:

1. At real time $t$, measure the current state $x(t)$.  
   Set it as the initial state $x_0^n$ for the SLQ problem.

2. Define a horizon of $N$ steps (or time $T$) and a cost
   $$
        \mathcal{J}
        = (\tilde{x}_N)^\top H \tilde{x}_N
        + \sum_{k=0}^{N-1}
            \big[
            \tilde{x}_k^\top Q\,\tilde{x}_k
            + \tilde{u}_k^\top R\,\tilde{u}_k
            + W(x_k,k)
            \big],
   $$
   where $\tilde{x}_k = x_k - x_k^{\text{ref}}$, $\tilde{u}_k = u_k - u_k^{\text{ref}}$, and $W$ may include waypoint penalties.

3. **Initialize the control law**:
   - Use an LQR/PD law or the solution from the **previous MPC step** as the initial $\{u_k^n\}$.

4. **Run a few SLQ iterations**:
   - Forward rollout, linearization, quadratization.  
   - Backward pass using Bellman (Riccati recursion, optimal affine increments $\delta u_k$).  
   - Line search update of $u_k^n$.

5. **Apply only the first control**:
   $$
    u_{\text{applied}}(t) = u_0^{\star}
   $$
   (possibly plus feedback $K_0(x(t)-x_0^n)$).

6. **Shift the horizon** and repeat at the next time step:
   - New state measurement $x(t+\Delta t)$.  
   - Use previous solution shifted in time as a warm start for $\{x_k^n,u_k^n\}$.  
   - Run SLQ again (usually only a few iterations needed if warm-started).

Additionally, as in the paper, you can:

- Use an **infinite-horizon LQR** around the goal state to compute a terminal cost matrix $H = P_{\infty}$, which serves as a good terminal cost for SLQ.
- This approximates the cost-to-go **beyond** the MPC horizon and improves stability.

Thus:

- **Inside each MPC step**: SLQ/iLQR uses **Bellman’s principle** and local quadratic approximations to compute a locally optimal affine law over the finite horizon.
- **Across time**: MPC uses the first control of that law, moves the horizon, and recomputes, leading to an online receding-horizon nonlinear controller.

---

## 11. Appendices

<details>
<summary>🔽 Appendix A: Line search and the role of feedback in SLQ/iLQR</summary>

You already have the local optimal **incremental** law from the backward pass:
$$
    \delta u_k^\star = l_k + K_k\,\delta x_k,
    \qquad \delta x_k = x_k - x_k^n.
$$

This is the solution of the **local** LQ subproblem (linearized dynamics + quadratized cost). Now we need to use it to update the **nominal control sequence** and generate a new trajectory on the **true nonlinear system**.

There are two closely-related notions here:

1. **Feedback vs feedforward**  
2. **Line search in the feedforward direction $\alpha$**

### A.1 Where does the feedback term go?

At iteration $i$, you have:

- Nominal trajectory: $\{x_k^n, u_k^n\}$
- New increments: $\delta u_k^\star = l_k + K_k\,\delta x_k$

When you **roll out** the new candidate trajectory on the nonlinear system, you don’t know in advance what the new state $x_k$ will be (because the dynamics are nonlinear). So you **use the policy** during the forward simulation:

1. Start from the actual initial state:
   $$
    x_0^{\text{cand}} = x_0^{\text{current}}.
   $$
2. At each time step $k$ of the rollout:

   - Compute the state deviation:
     $$
        \delta x_k = x_k^{\text{cand}} - x_k^n.
     $$
   - Compute the control *increment* using the affine law:
     $$
        \delta u_k = l_k + K_k\,\delta x_k.
     $$
   - Update the control:
     $$
        u_k^{\text{cand}} = u_k^n + \delta u_k.
     $$
   - Apply $u_k^{\text{cand}}$ to the **nonlinear** dynamics and get
     $$
        x_{k+1}^{\text{cand}} = f\big(x_k^{\text{cand}}, u_k^{\text{cand}}\big).
     $$

So in full, **with no line search**, the candidate control during rollout is:
$$
    u_k^{\text{cand}} = u_k^n + l_k + K_k \big(x_k^{\text{cand}} - x_k^n\big).
$$

> 🔹 The **feedback term** $K_k(x_k^{\text{cand}} - x_k^n)$ is *always present* in the forward rollout.  
> 🔹 The SLQ backward pass gives you both $l_k$ and $K_k$; you use the full law during rollout.

When we later introduce the step size $\alpha$, **we only scale the feedforward part $l_k$**, not the feedback:

$$
    u_k^{\text{cand}}(\alpha)
    = u_k^n + \alpha\, l_k + K_k\big(x_k^{\text{cand}}(\alpha) - x_k^n\big).
$$

- $K_k$ stays unchanged (it’s the local stabilizing feedback for the LQ subproblem).
- $\alpha$ scales the **step size** in the direction of the feedforward increment $l_k$.

### A.2 What is $\alpha$ and what is “line search”?

After the backward pass, you have a **search direction** in control space, given by the sequence $\{l_k\}_{k=0}^{N-1}$:

- Think of all $u_k$ stacked into a big vector $u$,
- All $l_k$ stacked into a big vector $l$,
- Then the new control candidate is
  $$
    u^{\text{cand}}(\alpha) = u^n + \alpha\,l \quad(\text{plus feedback corrections in rollout}).
  $$

This is exactly the idea of **line search** in optimization:

> We move along a direction $l$ from our current point $u^n$, and we choose how far to move by tuning the scalar step size $\alpha > 0$.

Concretely:

1. You start with $\alpha = 1$ (a “full” step).
2. You perform a forward rollout using
   $$
    u_k^{\text{cand}}(\alpha)
    = u_k^n + \alpha\, l_k + K_k\big(x_k^{\text{cand}}(\alpha) - x_k^n\big).
   $$
3. You compute the new cost $\mathcal{J}(\alpha)$ by summing the stage and terminal costs along the candidate trajectory.
4. If $\mathcal{J}(\alpha) < \mathcal{J}_{\text{old}}$ with sufficient decrease, **accept** this $\alpha$.
5. If not, **reduce** $\alpha$ (*e.g.* $\alpha \leftarrow \beta \alpha$ with $\beta \in (0,1)$, such as $\beta=0.5$) and try again.

This is called a **backtracking line search**. Formally:

- We are minimizing the scalar function
  $$
    \phi(\alpha) := \mathcal{J}\big(u^n + \alpha\,l\big)
  $$
  (with feedback and dynamics included in how we evaluate $\mathcal{J}$).
- We want an $\alpha$ that gives **actual decrease in cost** for the **true nonlinear problem**, not just the local quadratic approximation.

### A.3 Why not always $\alpha = 1$?

Intuitively:

- The backward pass (Riccati step) gives an **optimal step for the quadratic approximation** of the cost and linearized dynamics.  
- But the **true problem is nonlinear**; the quadratic approximation may be poor if we step too far from the nominal trajectory.
- A full step $\alpha = 1$ can **overshoot**: the new rollout might increase the cost or even destabilize the system.

So:

- If the model is very nonlinear or the initial guess is far from optimal, $\alpha = 1$ may be too aggressive.
- A smaller $\alpha$ takes a **shorter step**, staying closer to the region where the local linear/quadratic approximation is valid.

In optimization language:

> The Riccati backward pass gives you a **descent direction** $l$.
> The line search on $\alpha$ ensures you get a **descent step** for the *true* cost.

In practice, people often try a sequence like:
$$
\alpha \in \{1,\, 0.5,\, 0.25,\, 0.1,\, 0.05,\dots\}
$$
and pick the **largest** $\alpha$ that produces a sufficient cost decrease.

### A.4 Why only scale $l_k$ and not $K_k$?

- $K_k$ is chosen to be the **optimal feedback** for the local LQ approximation. It stabilizes deviations around the nominal and shapes how the state trajectory responds to disturbances during the rollout.
- $l_k$ is the **feedforward step** that shifts the nominal control in the direction of lower cost.

Scaling both $l_k$ and $K_k$ would break the carefully derived LQ structure. The standard (and widely used) iLQR/SLQ practice is:

- **Keep $K_k$** as is (full feedback),
- **Scale only $l_k$** by $\alpha$.

So the update is:

- Incremental law from backward pass:
  $$
    \delta u_k^\star = l_k + K_k\,\delta x_k.
  $$
- Line search candidate during rollout:
  $$
    u_k^{\text{cand}}(\alpha)
    = u_k^n + \alpha\, l_k + K_k\big(x_k^{\text{cand}}(\alpha) - x_k^n\big).
  $$

### A.5 Summary

- The **feedback term** $K_k(x_k - x_k^n)$ is always used during the forward rollout to stabilize and correct the trajectory. It is **not** scaled by $\alpha$.
- The **feedforward term** $l_k$ is the “direction” in control space given by the local LQ solution; we scale it by a step size $\alpha$.
- **Line search** = choose $\alpha$ by trying $\alpha=1,0.5,0.25,\dots$ and picking the largest one that actually **reduces the true cost** $\mathcal{J}$ when the nonlinear system is rolled out.
- This is how SLQ/iLQR combines:
  - a **local quadratic optimization** step (backward pass)  
  - with a **global cost check** on the true nonlinear problem (line search),  
  ensuring stable, monotonic convergence in practice.

</details>


<details>
<summary>🔽 Appendix B: Influence of terminal cost and state cost gradient</summary>

### B.1 Terminal cost $\rightarrow$ boundary condition $\rightarrow$ whole policy

Recall the quadratic expansion of the terminal cost:
$$
    h(x_N)
    \approx h_N^n
    + h_{x,N}^\top \delta x_N
    + \tfrac12\,\delta x_N^\top H_N \delta x_N.
$$

This defines the **terminal value function**:
$$
    V_N(\delta x_N)
    \approx s_N + p_N^\top \delta x_N
    + \tfrac12\,\delta x_N^\top P_N \delta x_N,
$$

with
$$
    P_N := H_N, \qquad p_N := h_{x,N}.
$$

Now look at the **last stage** $k = N-1$. The matrices there are
$$
    \begin{aligned}
    & H_{N-1} = R_{N-1} + B_{N-1}^\top P_N B_{N-1}, \\
    & G_{N-1} = B_{N-1}^\top P_N A_{N-1}, \\
    & g_{N-1} = r_{N-1} + B_{N-1}^\top p_N.
    \end{aligned}
$$

The control law at time $N-1$ is
$$
    \delta u_{N-1}^\star
    = l_{N-1} + K_{N-1} \delta x_{N-1},
$$

with
$$
    K_{N-1} = -H_{N-1}^{-1} G_{N-1}, \qquad
    l_{N-1} = -H_{N-1}^{-1} g_{N-1}.
$$

So you can see **explicitly**:

- $P_N$ appears in $H_{N-1}, G_{N-1} \Rightarrow$ changes **feedback** $K_{N-1}$.

- $p_N$ appears in $g_{N-1} \Rightarrow$ changes **feedforward** $l_{N-1}$.

Then we update the value function at time $N-1$:
$$
    \tilde{Q}_{N-1} := Q_{N-1} + A_{N-1}^\top P_N A_{N-1},
$$
$$
    \tilde{q}_{N-1} := q_{N-1} + A_{N-1}^\top p_N,
$$
and
$$
    \begin{aligned}
    P_{N-1}
    &= \tilde{Q}_{N-1}
    + K_{N-1}^\top H_{N-1} K_{N-1}
    + K_{N-1}^\top G_{N-1}
    + G_{N-1}^\top K_{N-1}, \\[0.3em]
    p_{N-1}
    &= \tilde{q}_{N-1}
    + K_{N-1}^\top H_{N-1} l_{N-1}
    + G_{N-1}^\top l_{N-1}
    + K_{N-1}^\top g_{N-1}.
    \end{aligned}
$$

Thus:

- $P_N, p_N$ determine $H_{N-1}, G_{N-1}, g_{N-1}$,
- which determine $K_{N-1}, l_{N-1}$,
- which determine $P_{N-1}, p_{N-1}$,

and then at step $k = N-2$ we use $P_{N-1}, p_{N-1}$ in exactly the same way. Repeating this backwards shows:

> The terminal cost sets the *boundary condition* $(P_N,p_N)$ of the value function, and via the Bellman recursion it *influences* $(K_k,l_k)$ at *every earlier* time step.

This is precisely why in MPC, a well-chosen terminal cost (*e.g.* from infinite-horizon LQR) can dramatically change the behavior of the whole finite-horizon controller.

### B.2 How the state cost gradient $q_k = \ell_{x,k}$ influences the policy

The term $q_k$ affects the control policy **indirectly** and its effect on the control law is global in time.

Recall the definitions:
$$
    q_k := \ell_{x,k}, \qquad
    \tilde{q}_k := q_k + A_k^\top p_{k+1}.
$$

The backward recursion for the value-function gradient is
$$
    p_k
    = \tilde{q}_k
    + K_k^\top H_k l_k
    + G_k^\top l_k
    + K_k^\top g_k.
$$

So the **chain of influence** from the state cost gradient is:

1. The stage cost gradient w.r.t. state at time $k$ is $q_k = \ell_{x,k}$.
2. It enters the “effective” linear term
   $$
       \tilde{q}_k = q_k + A_k^\top p_{k+1}.
   $$
3. $\tilde{q}_k$ contributes directly to the value-function gradient at time $k$:
   $$
       p_k
       = \tilde{q}_k
       + K_k^\top H_k l_k
       + G_k^\top l_k
       + K_k^\top g_k.
   $$
4. At the **previous time step** $k-1$, the gradient $p_k$ appears in
   $$
       g_{k-1} = r_{k-1} + B_{k-1}^\top p_k,
   $$
   and then
   $$
       l_{k-1} = -H_{k-1}^{-1} g_{k-1}.
   $$

So:

- $q_k$ does **not** enter the current step’s $g_k$ **directly**:
  $$
      g_k = r_k + B_k^\top p_{k+1}.
  $$
- Instead, $q_k$ shapes the **value function** via $p_k$, and this updated $p_k$ then affects **earlier** controls through $g_{k-1}$ and $l_{k-1}$.

Intuitively:

- $q_k$ tells you ***“how much do I dislike being in this state at time $k$”***.  
- That information is stored in the value-function gradient $p_k$.  
- Dynamic programming then propagates this information backward in time, and through the $g_{k-1}, l_{k-1}$ terms it ultimately changes the entire feedback policy $\{K_j, l_j\}_{j=0}^{N-1}$.

</details>


<details>
<summary>🔽 Appendix C: Waypoint Cost in SLQ-MPC</summary>

### C.1 Waypoint Cost $W(x_k,k)$ in SLQ-MPC

In the literature<sup>[1](#icra16)</sup> an intermediate (stage) waypoint cost is introduced to penalize the deviation between desired and current state, with penalties concentrated around specific **time steps** where each waypoint is assigned.

Recall that the overall finite-horizon cost is
$$
    \mathcal{J}
    = (\tilde{x}_N)^\top H \tilde{x}_N
    + \sum_{k=0}^{N-1}
        \big[
        \tilde{x}_k^\top Q\,\tilde{x}_k
        + \tilde{u}_k^\top R\,\tilde{u}_k
        + W(x_k,k)
        \big],
$$
where
$$
    \tilde{x}_k = x_k - x_k^{\text{ref}}, \qquad
    \tilde{u}_k = u_k - u_k^{\text{ref}}
$$
are deviations from a desired state/input trajectory.

The **waypoint cost** $W(x_k,k)$ is the discrete-time analogue of their continuous-time term:
$$
    W(x_k,k)
    = \sum_{n=0}^{N_{\text{wp}}-1}
    \hat{x}_{n,k}^\top W_{p,n}\,\hat{x}_{n,k}\;
    \sqrt{\frac{\rho_{p,n}}{2\pi}}\,
    \exp\!\Big(
    -\frac{\rho_{p,n}}{2}\,(k - k_{p,n})^2
    \Big),
$$
where

- $n = 0,\dots,N_{\text{wp}}-1$: waypoint index,  
- $\hat{x}_{n,k}$: deviation from the $n$-th waypoint state at time step $k$, *e.g.*  
  $$\hat{x}_{n,k} = x_k - x_n^{\text{wp}},$$  
- $W_{p,n}$: waypoint cost matrix (which state components are important at waypoint $n$),  
- $k_{p,n}$: desired **time step** at which waypoint $n$ should be reached,  
- $\rho_{p,n}$: temporal sharpness (inverse variance of the Gaussian in the time index).

> Each waypoint contributes a *quadratic penalty in state* multiplied by a *Gaussian bump in time index* centered at $k_{p,n}$.

### C.2 Intuition

- The term $\hat{x}_{n,k}^\top W_{p,n}\hat{x}_{n,k}$ penalizes **state error** to waypoint $n$:
  $$
    \hat{x}_{n,k} = x_k - x_n^{\text{wp}}.
  $$
- The Gaussian factor
  $$
    \sqrt{\frac{\rho_{p,n}}{2\pi}}
    \exp\!\Big(-\tfrac{\rho_{p,n}}{2}(k - k_{p,n})^2\Big)
  $$
  activates this penalty **only near** time step $k_{p,n}$, fading out before and after.

So at each time step $k$:

- The **trajectory-tracking error**
  $$
    \tilde{x}_k = x_k - x_k^{\text{ref}}
  $$
  measures “How far am I from the **main reference trajectory** right now?” and is penalized by $\tilde{x}_k^\top Q \tilde{x}_k$ for **all** $k$.

- The **waypoint error**
  $$
    \hat{x}_{n,k} = x_k - x_n^{\text{wp}}
  $$
  measures “How far am I from **waypoint $n$’s state**?”, but is weighted in time by the Gaussian, so it matters **only near** $k_{p,n}$.

In other words:

- $\tilde{x}_k$ pulls the trajectory toward the **global reference** over the whole horizon.  
- $\hat{x}_{n,k}$ adds extra pull toward **specific waypoint states** at **specific times**.

**Tuning:**

- Larger $W_{p,n}$ $\rightarrow$ waypoint $n$ is more important in state space.  
- Larger $\rho_{p,n}$ $\rightarrow$ **narrower** time window (more precise timing around $k_{p,n}$).  
- Smaller $\rho_{p,n}$ $\rightarrow$ looser requirement on when the waypoint is reached.

**Potential concern (double-counting the same objective):**

If the reference trajectory $x_k^{\text{ref}}$ already passes through the waypoints at the desired times (*i.e.* $x_{k_{p,n}}^{\text{ref}} \approx x_n^{\text{wp}}$), then near $k_{p,n}$ the two errors
$$
  \tilde{x}_k = x_k - x_k^{\text{ref}}, \qquad
  \hat{x}_{n,k} = x_k - x_n^{\text{wp}}
$$
are almost the same. In that case, the trajectory cost $\tilde{x}_k^\top Q \tilde{x}_k$ and the waypoint cost $\hat{x}_{n,k}^\top W_{p,n}\hat{x}_{n,k}$ **both pull toward essentially the same target**, and their weights effectively add up.

If $Q$ and $W_{p,n}$ (and the temporal sharpness $\rho_{p,n}$) are chosen too large simultaneously, the combined tracking + waypoint penalties can become **dominant** over other important terms (*e.g.* input regularization, obstacle costs, soft constraints). In practice, this means you must **carefully tune**:

- the magnitude of $Q$ vs. $W_{p,n}$, and  
- how sharply in time the waypoint penalty is activated via $\rho_{p,n}$,

to avoid over-emphasizing waypoints relative to the rest of the cost and to reflect the true task priorities.


### C.3 Why this form is useful

1. **Soft waypoint constraints**

   Instead of hard constraints $x_{k_{p,n}} = x_n^{\text{wp}}$, they use a cost term.  
   This lets the optimizer:
   - strongly **encourage** passing near the waypoint,
   - but still **violate** it slightly if dynamics or input limits require it.

2. **Quadratic in state $\rightarrow$ SLQ-friendly**

   For each fixed $k$, $W(x_k,k)$ is quadratic in $\hat{x}_{n,k}$.  
   The time-dependent Gaussian is just a scalar weight, so the cost remains:
   - quadratic in state,
   - smooth in the time index.  
   This fits perfectly into SLQ’s linear–quadratic approximation framework.

3. **Unifies trajectory tracking and waypoint shaping**

   - The usual $Q,R,H$ terms handle final/trajectory tracking via $\tilde{x}_k,\tilde{u}_k$.
   - $W(x_k,k)$ adds **intermediate landmarks** (gates, windows, obstacle clearances, etc.).  
   Both are handled uniformly as running/terminal quadratic costs.

4. **Flexible tuning**

   Different waypoints can have different $W_{p,n}$ and $\rho_{p,n}$, allowing:
   - emphasis on certain state components at specific waypoints,
   - precise timing for some waypoints and relaxed timing for others.

> The waypoint cost $W(x_k,k)$ is a *time-localized quadratic penalty* that softly pulls the trajectory toward specified intermediate states at specified time steps, while preserving the quadratic structure needed by SLQ. It enforces waypoints “as soft constraints” without making the problem hard or infeasible.

</details>
