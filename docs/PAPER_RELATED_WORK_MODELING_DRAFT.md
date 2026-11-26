# Manuscript draft — Related Work and State-Conditioned Comfort Modeling

> Drafted 2026-11-26 for incorporation into the research paper. This is a **writing draft**, not an existing submitted manuscript. The repository currently contains no `.tex`, `.bib`, or manuscript document. Experimental code and historical results are unchanged.
>
> Evidence labels: **[H–H empirical]** = human–human study; **[H–R empirical]** = human–robot study; **[transfer]** = transfer to a different task/robot; **[hypothesis]** = proposed modeling choice, not a measured comfort boundary. Do not describe simulation outputs as human-validated comfort.

## Suggested manuscript placement

- **Related Work / Human-aware Navigation:** Gorrini (2014, 2019), Neggers (2022 speed/passing), Neggers (2022 direction).
- **Related Work / Task-dependent Proxemics:** Kinoe and Mizuno (2015), Kinoe (2018), Cui et al. (2023).
- **Proposed Method / Comfort Space Modeling:** human state, relative direction, motion heading, speed, and task-related uncertainty; offset ellipse.
- **Experiments / Stage 2:** literature-predicted comfort threshold and minimum *policy extent* calibrated using observed trajectories; see `docs/STAGE2_LOWER_BOUND_REDESIGN.md`.

## Ready-to-use English text: Related Work

### Locomotion- and direction-dependent proxemics

Personal space is context dependent rather than a fixed, isotropic radius. Gorrini et al. (2014) investigated the frontal personal-space zone of pedestrians under stationary and walking conditions in controlled human–human encounters. Building on the same experiment conducted in 2013, Gorrini et al. (2019) described the measured changes in frontal interpersonal clearance and employed proxemic representations in a simulation-based crowding and stress model. The experimental data indicate that locomotion and encounter speed influence preferred frontal separation. Importantly, the recorded frontal distances across walking-speed conditions did not increase monotonically at every tested speed; they therefore motivate speed sensitivity without establishing a universal linear mapping for human–robot encounters. The two articles should not be counted as independent human experiments. [Gorrini2014; Gorrini2019]

In human–robot navigation, Neggers et al. (2022a) empirically measured comfort ratings when stationary participants were passed by a humanoid robot or a custom automated guided vehicle from different directions. The resulting comfort–distance relationships were direction dependent, with rear passages generally experienced as less comfortable than frontal passages. Neggers et al. (2022b) subsequently examined robot passing and overtaking of walking participants and reported that perceived comfort varied with distance, robot speed, and encounter type; overtaking was less comfortable than passing under the tested conditions. These findings support a state- and encounter-aware representation of proxemic requirements. However, the available fitted walking-person curve does not itself establish a distinct front, side, and rear comfort contour for a moving pedestrian in an industrial multi-AMR environment. [Neggers2022Shape; Neggers2022Speed]

### Task-dependent interpersonal distance

Interpersonal spacing is also sensitive to activity and social interaction. Kinoe and Mizuno (2015) found differences in preferred interpersonal distance between no-task and task conditions, as well as interactions with body orientation. Kinoe (2018) further found that cooperative task initiation shortened preferred interpersonal distance between interacting participants. In a controlled study, Cui et al. (2023) found reduced personal space following cooperative social interaction, while the corresponding solo-task control group did not show a significant reduction. These results establish that task context and social interaction can modulate interpersonal proximity, but do not demonstrate that an industrial worker engaged with a machine necessarily tolerates closer **rear** passage by an unrelated AMR. Accordingly, task-specific contraction is treated in this work as a hypothesis to be examined through sensitivity analysis rather than an established human–robot comfort constraint. [KinoeMizuno2015; Kinoe2018; Cui2023]

### Research gap

The cited studies separately examine pedestrian locomotion, robot passing distance and speed, and the situational regulation of interpersonal space. They do not jointly provide experimentally validated nine-cell comfort thresholds for the product of three human states (moving, stationary, station engaged) and three relative AMR directions (front, side, rear) in a narrow industrial aisle with multiple robots. We therefore distinguish (i) literature-predicted comfort requirements available for empirically studied encounters, (ii) explicit transfer assumptions for unsupported state–direction combinations, and (iii) the operational comfort-space configuration required by an actual navigation stack to realize those separations. The latter is identified by simulation but is not itself a measurement of subjective comfort.

## Ready-to-use English text: Proposed Modeling

### State- and direction-conditioned geometry

For person `i`, let `H_i(t)` denote the human state and `theta_i(t)` denote their heading. The proposed comfort geometry is parameterized by three extents, `F_i(H_i,v_i)`, `S_i(H_i,v_i)`, and `R_i(H_i,v_i)`, measured from the human center toward the front, lateral direction, and rear, respectively. An offset ellipse is constructed using semi-axes `a_i=(F_i+R_i)/2`, `b_i=S_i`, and center displacement `Delta_i=(F_i-R_i)/2` along the human heading. For moving people the orientation follows the measured velocity direction; for stationary people it follows body orientation when available. This parameterization supports both forward-extended and rear-extended regions using the existing navigation stack. 

We ground the **stationary** directional ordering in human–robot comfort measurements (Neggers et al., 2022a). For **moving** people, the use of velocity-aligned, potentially forward-extended space is motivated by pedestrian locomotion experiments (Gorrini et al., 2014/2019) and robot passing studies (Neggers et al., 2022b); the magnitude of any speed-dependent forward expansion is explicitly an engineering hypothesis until robot-specific directional preference data become available. For **station-engaged** people, studies of activity-related interpersonal spacing (Kinoe and Mizuno, 2015; Kinoe, 2018; Cui et al., 2023) motivate the inclusion of task as a contextual factor, but not a predetermined shrinkage in any direction. In the evidence-constrained primary model, their comfort–distance reference is therefore the stationary-person reference; optional engagement-specific adjustments are reserved for exploratory sensitivity analysis and should not be interpreted as validated human comfort.

### Two distinct lower bounds

Stage 2 separates the **literature-referenced minimum physical clearance**, `D_req(H,phi;C*)`, from the **minimum configured comfort extent**, `R_policy_min(H,phi)`. The first is obtained by inverting an external human-rating curve at a predeclared target `C*` on its original distance basis and within its empirical domain. The second is the smallest candidate comfort-space extent that yields sufficiently many **completed, collision-free passages meeting the predicted comfort target** in an identical fixed-geometry simulation scenario. A configured boundary is not assumed equal to achieved clearance; actual robot footprints and trajectories are measured. Published curves and threshold choices are not presented as new human experiments.

Where existing literature provides no task-specific or encounter-specific comfort curve (e.g., a machine-engaged worker or the front/rear of a moving person), the model either uses a disclosed conservative transfer baseline or labels the cell as unresolved. It does not infer a human comfort threshold from navigation success or from a chosen comfort-zone radius.

## Citation evidence and caveats

| Source | Empirical scope | What can be claimed | What cannot be claimed |
|---|---|---|---|
| Gorrini et al. (2014) | Human–human, static/walking frontal spacing | Locomotion affects frontal interpersonal space | Calibrated industrial AMR front radius |
| Gorrini et al. (2019) | Reuses 2013 Tokyo study, builds crowd/stress model | Speed-dependent and nonmonotone reported frontal distances | Independent replication of 2014; exact AMR clearance |
| Neggers et al. (2022a) | Standing humans; robot pass front/back/left/right, 0.35 m/s | Direction-dependent human–robot comfort | Machine-engaged worker's reduced rear distance |
| Neggers et al. (2022b) | Walking humans; robot passing/overtaking and speed | Distance, speed, encounter type affect comfort | Fully measured moving front/side/rear footprint |
| Kinoe and Mizuno (2015) | Human–human, no task vs smartphone/music tasks | Situational task conditions alter preferred spacing | Universal task-rear shrinkage around AMRs |
| Kinoe (2018) | Human–human cooperative dyads | Cooperative task initiation can reduce preferred dyadic distance | Reduced distance to a non-cooperating industrial AMR |
| Cui et al. (2023) | Cooperative vs solo-task control; 115 participants | Cooperative social interaction can reduce personal space; solo-task control did not | All forms of solitary task engagement reduce personal space |

## References to import into the paper bibliography

1. Gorrini, A., Shimura, K., Bandini, S., Ohtsuka, K., & Nishinari, K. (2014). Experimental Investigation of Pedestrian Personal Space: Toward Modeling and Simulation of Pedestrian Crowd Dynamics. *Transportation Research Record*, 2421, 57–63. https://doi.org/10.3141/2421-07
2. Gorrini, A., Crociani, L., Vizzari, G., & Bandini, S. (2019). Stress estimation in pedestrian crowds: Experimental data and simulations results. *Web Intelligence*, 17(1). https://doi.org/10.3233/WEB-190403
3. Neggers, M. M. E., Cuijpers, R. H., Ruijten, P. A. M., & IJsselsteijn, W. A. (2022a). Determining Shape and Size of Personal Space of a Human when Passed by a Robot. *International Journal of Social Robotics*, 14, 561–572. https://doi.org/10.1007/s12369-021-00805-6
4. Neggers, M. M. E., Cuijpers, R. H., Ruijten, P. A. M., & IJsselsteijn, W. A. (2022b). The effect of robot speed on comfortable passing distances. *Frontiers in Robotics and AI*, 9, 915972. https://doi.org/10.3389/frobt.2022.915972
5. Kinoe, Y., & Mizuno, N. (2015). Situational Transformation of Personal Space. In *Human Interface and the Management of Information: Information and Knowledge in Context*, LNCS 9173, pp. 15–24. https://doi.org/10.1007/978-3-319-20618-9_2
6. Kinoe, Y. (2018). Dynamic Adjustment of Interpersonal Distance in Cooperative Task. In *Proceedings of the 20th Congress of the International Ergonomics Association*, AISC 827, pp. 340–349. https://doi.org/10.1007/978-3-319-96059-3_37
7. Cui, S., Yang, T., & Liu, N. (2023). Generalization of the modulatory effect of social interaction on personal space. *Frontiers in Psychology*, 14, 1148395. https://doi.org/10.3389/fpsyg.2023.1148395

**Editorial checks before submission:** finalize the citation-key scheme and bibliographic style in the actual manuscript; do not imply that Gorrini (2014) and (2019) are independent human experiments; do not claim that a predicted rating obtained in simulation is a measured subjective rating.
