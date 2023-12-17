package dev.aether.ktftc.annotations


@MustBeDocumented
@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
annotation class KtAutonomous(
    /**
     * The name to be used on the driver station display. If empty, the name of
     * the OpMode class will be used.
     * @return the name to use for the OpMode in the driver station.
     */
    val name: String = "",
    /**
     * Optionally indicates a group of other OpModes with which the annotated
     * OpMode should be sorted on the driver station OpMode list.
     * @return the group into which the annotated OpMode is to be categorized
     */
    val group: String = "",
    /**
     * The name of the TeleOp OpMode you'd like to have automagically preselected
     * on the Driver Station when selecting this Autonomous OpMode. If empty, then
     * nothing will be automagically preselected.
     *
     * @return see above
     */
    val preselectTeleOp: String = ""
)

