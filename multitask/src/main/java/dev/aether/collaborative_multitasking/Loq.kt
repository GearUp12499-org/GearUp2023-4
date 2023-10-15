package dev.aether.collaborative_multitasking

/**
 * it's a Loq! a "lock" but without any of the atomic magic
 */
class Loq(private val id: String) {
    override fun hashCode(): Int {
        return id.hashCode()
    }

    override fun equals(other: Any?): Boolean {
        return when (other) {
            (other == null) -> false
            is String -> id == other
            is Loq -> id == other.id
            else -> false
        }
    }
}