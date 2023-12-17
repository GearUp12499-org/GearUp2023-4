package dev.aether.ktftc.annotations

/**
 * Add this annotation to an OpMode to add it to the OpMode list, unless it is also @KtDisabled.
 * @see KtDisabled
 */

@MustBeDocumented
@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
annotation class KtTeleOp(
    /**
     * The name to be used on the driver station display. If empty, the name of
     * the OpMode class will be used.
     * @return the name to use for the OpMode on the driver station
     */
    val name: String = "",
    /**
     * Optionally indicates a group of other OpModes with which the annotated
     * OpMode should be sorted on the driver station OpMode list.
     * @return the group into which the annotated OpMode is to be categorized
     */
    val group: String = ""
)

