**Project Overview**

![Image 1](https://github.com/smarahmati/CatForelimbNeuromechanics/blob/master/Graphical%20abstract.png)
![Image 2](https://github.com/smarahmati/CatForelimbNeuromechanics/blob/master/Cat%20forelimb%20musculoskeletal%20model%20figure.png)
![GIF](https://github.com/smarahmati/CatForelimbNeuromechanics/blob/master/Cat%20locomotion%20simulation.gif)
![GIF](https://github.com/smarahmati/CatForelimbNeuromechanics/blob/master/Cat%20forelimb%20musculoskeletal%20model%20simulation.gif)



-   **Title:** ROLE OF FORELIMB MORPHOLOGY IN MUSCLE SENSORIMOTOR
    FUNCTIONS DURING LOCOMOTION IN THE CAT

-   **Components:** 13 parts providing a comprehensive model of the
    cat\'s forelimb neuromechanics.

**General Notes**

-   Main scripts begin with \'A\_\', \'B\_\', etc., and must be run in
    sequence.

-   Abbreviations:

    -   \"OW\" and \"OV\" (same): Muscle Origin to Muscle Via-point

    -   \"WI\" and \"VI\" (same): Muscle Via-point to Muscle Insertion

**Description of Each Part**

**A) Muscle Length and Moment Arm**

-   **A1 - Musculoskeletal data**

    -   Scripts:

        -   [A_MusculoskeletalSystem_SagittalPlane](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/A%29%20Muscle%20length%20and%20moment%20arm/A1%20-%20Musculoskeletal%20data):
            Loads data from Excel sheets containing your experimental
            data (musculoskeletal information). It performs the
            following:

            -   Extracts muscle origin and insertion points, joint
                centers, and other relevant anatomical data.

            -   Fits a plane to the attachment points using Principal
                Component Analysis (PCA) and projects anatomical
                landmarks for accurate spatial representation.

            -   Plots the muscles and segments in a 3D figure,
                visualizing the fitted plane, annotating muscles and
                attachment points, and highlighting key anatomical
                features.

        -   [B_Musculoskeletal_Visualization_2D_3D](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/A%29%20Muscle%20length%20and%20moment%20arm/A1%20-%20Musculoskeletal%20data):
            Performs 2D and 3D visualization of selected muscles.
            Computes rotations to align anatomical points on the
            computed sagittal plane with the XY plane. It also:

            -   Computes muscle lengths and geometrical parameters.

            -   Estimates segment lengths and angles.

            -   Creates a structure named \'MusculoskeletalData\' for
                further analysis.

-   **A2 - Computation of muscle length and moment arm**

    -   Script:
        [A_RunAll](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/A%29%20Muscle%20length%20and%20moment%20arm/A2%20-%20Computation%20of%20muscle%20length%20and%20moment%20arm)
        processes all sub-scripts related to individual muscles. It
        performs the following:

        -   Scales musculotendon attachment points.

        -   Computes musculotendon length, velocity, and moment arm
            during locomotion for each muscle.

        -   Generates two structures: \'MuscleLengthVelocityMomentArm\'
            and \'ScaledMuscleMorphologicalParameters\' for further
            analysis.

**B) Muscle mechanical properties**

-   Script:
    [A_Muscle_Mechanical_Properties](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/B%29%20Muscle%20mechanical%20properties)
    creates a table named \'MuscleMP\' containing muscle mechanical
    properties derived from our experimental data in the Excel sheet
    titled \'ForelimbArchitecture\'. This table will be used for further
    analysis.

**C) Muscle dynamics based on fixed tendon length computed at maximum
musculotendon length (MTL)**

-   Script:
    [A_Maximum_Muscle_Force_Moment_Computation](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/C%29%20Muscle%20dynamics%20based%20on%20fixed%20tendon%20length%20computed%20at%20maximum%20MTL)
    computes the maximum musculotendon forces and moments resulting from
    maximum activation during locomotion. The results are saved in a
    structure array \'MaxMuscleForceMoment\'. It uses input data
    including:

    -   Cycle time from \'MotionData\'

    -   MT length, velocity, and moment arm data from
        \'MuscleLengthVelocityMomentArm\'

    -   Muscle mechanical parameters from \'MuscleMP\'

-   Script:
    [B_Plot_3in3_figs](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/C%29%20Muscle%20dynamics%20based%20on%20fixed%20tendon%20length%20computed%20at%20maximum%20MTL)
    calculates the biomechanical variables of a selected MT unit during
    locomotion based on input data including:

    -   Cycle time from \'MotionData\'

    -   MT length, velocity, moment arms from
        \'MuscleLengthVelocityMomentArm\'

    -   Muscle mechanical parameters from \'MuscleMP\'

<!-- -->

-   Generates plots to visualize these variables over a locomotion
    cycle.

**D) Muscle clustering based on constant tendon length computed at
maximum MTL**

-   Script:
    [A_Muscle_Clustering](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/D%29%20Muscle%20clustering%20based%20on%20constant%20tendon%20length%20computed%20at%20maximum%20MTL)
    performs k-means clustering on maximum muscle moments and compares
    it with function-based grouping. It generates the \'MuscleCluster\'
    cell array, which shows the cluster of muscles in each cell.

**E) Mechanical properties of 9 muscle groups**

-   Script:
    [A_Muscle_Mechanical_Properties_9_Groups](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/E%29%20Mechanical%20properties%20of%209%20muscle%20groups)
    calculates and summarizes the mechanical properties of 9 muscle
    groups based on the clustering results and mechanical properties of
    40 muscles. It computes:

    -   Mean for each equivalent muscle related to each group (except
        mass and PCSA parameters, which are summed).

**F) Muscle Morphology Optimization (9 Muscle Groups)**

-   **F1) Initial Morphologic Parameters of 9 Muscle Groups**

    -   Script:
        [A_InitialMusculoskeletalData_9_Groups](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/F1%29%20Initial%20Morphologic%20parameters%20of%209%20muscle%20groups)

        -   Generates initial morphological parameters (\'a\' and
            \'phi\') for 9 muscle groups.

        -   Includes joint radius ranges based on individual muscles in
            each group.

        -   Computes ranges for other morphological parameters
            (including \'a\' and \'phi\') from SolidWorks drawings.

        -   Saves generated data in \'NineMuscleMorphParaInitial\' for
            further analysis.

-   **F2) Optimization to Tune 9 Muscle Groups Morphological
    Parameters**

    -   This folder tunes the geometric parameters to match the maximum
        moment of the original muscle groups.

    -   It uses an interior point optimization algorithm and includes 10
        subfolders:

        -   Folders 1-9: Each optimizes a specific muscle group.

        -   Consolidated folder: Combines the results of all groups.

    -   Script:
        [A_RunAll](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/F2%29%20Optimization%20to%20tune%209%20muscle%20groups%20morphological%20parameters/Creating%20MT_LVMA_9Groups%2C%20MaxMTFM_9Groups%20%26%20MTData_9Groups)
        (within the consolidated folder)

        -   Creates:

            -   MuscleLengthVelocityMomentArm_9Groups: Musculotendon
                length, velocity, and moment arm during locomotion (for
                9 groups).

            -   MusculoskeletalData_9Groups: Joint centers, muscle
                origins/insertions, and geometric parameters during
                locomotion (for 9 groups).

            -   MaxMuscleForceMoment_9Groups: Musculotendon force,
                moment, fascicle length, tendon length, etc. during
                locomotion with maximum activation (for 9 groups).

**G) Musculoskeletal System Animation**

-   This folder contains two subfolders for animating the cat\'s
    forelimb during locomotion:

    -   **Subfolder 1:** Animation with joint angular velocities and
        moments displayed as circular arrows.

    -   **Subfolder 2:** Animation of the musculoskeletal system without
        additional details.

**H) Muscle Activation Computation**

-   **H1) Computation of Muscle Activations for 40 Muscles**

    -   Script:
        [A_Optimization](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/H1%29%20Computation%20of%20muscle%20activations%20for%2040%20muscles)

        -   Minimizes muscle fatigue for 40 muscles using a cost
            function.

        -   Optimization parameters are muscle activations.

        -   Employs static optimization with random initial conditions
            for stable convergence.

        -   Constrained by Constraint_Function to ensure joint moments
            equal the sum of corresponding muscle moments.

    -   Script:
        [B_Results](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/H1%29%20Computation%20of%20muscle%20activations%20for%2040%20muscles)

        -   Loads optimal muscle activations from \'Activations.mat\'
            (created by A_Optimization).

        -   Computes muscle forces and moments, calculates the sum of
            muscle moments at each joint using Results_Function.

        -   Compares joint moments with the sum of corresponding muscle
            moments for equality.

        -   Generates plots for user-selected muscle activation.

-   **H2) Computation of Muscle Activations for 9 Muscles** (Similar
    process to H1)

    -   Script:
        [A_Optimization](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/H2%29%20Computation%20of%20muscle%20activations%20for%209%20muscles)
        (for 9 equivalent muscles)

    -   Script:
        [B_Results](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/H2%29%20Computation%20of%20muscle%20activations%20for%209%20muscles)
        (for 9 equivalent muscles)

**I) Comparison of EMG Data and Computed Activations**

-   Script:
    [A_EMGs_Individual_Combined_Muscles_WithSecondAxis](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/I%29%20EMGs%20vs%20computed%20activations)

    -   Processes and compares muscle activations from individual
        muscles and their equivalent muscles with EMG data.

**J) Sensory Feedback Computation**

-   Script:
    [A_Sensory_Feedbacks](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/J%29%20Computation%20of%20sensory%20feedback)

    -   Processes and analyzes sensory feedback in cat locomotion (Ia,
        II, and Ib afferent activities).

    -   Uses muscle activations, muscle-tendon lengths/velocities/forces
        (for individual muscles and equivalent groups).

    -   Saves normalized results for further analysis.

-   Script:
    [B_Spinal_Map_Sensory_Feedbacks](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/J%29%20Computation%20of%20sensory%20feedback)

    -   Computes a spinal map of motor and sensory neuron activities
        during the walking cycle.

    -   Uses proportional motor pool distribution from literature.

**K) Sensitivity Analysis**

-   Performed in 9 separate subfolders (Sensitivity Analysis Group 1 to
    9).

-   Analyzes how changes in major physiological/mechanical and
    geometrical parameters affect maximum moment of equivalent muscle.

-   Each subfolder includes a script:
    [A_MT_Sobol_Sensitivity_Analysis](https://github.com/smarahmati/CatForelimbNeuromechanics/tree/master/Codes/K%29%20Sensitivity%20analysis/Sensitivity%20Analysis%20Group%201)

    -   Computes sensitivity of mean maximum MT moment to muscle
        parameter changes.

    -   Uses Sobol method implemented in SobolGSA software (Kucherenko &
        Zaccheus, 2016).

    -   Requires downloading and installing the Global Sensitivity
        Analysis Toolbox
        (<https://www.mathworks.com/matlabcentral/fileexchange/40759-global-sensitivity-analysis-toolbox>).

-   Sensitivity analysis results can be plotted in the corresponding
    \'GraphSensitivity\' subfolder.
