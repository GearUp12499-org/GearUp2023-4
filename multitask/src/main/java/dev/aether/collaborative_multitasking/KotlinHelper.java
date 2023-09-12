package dev.aether.collaborative_multitasking;

import java.util.function.Consumer;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;
import kotlin.jvm.functions.Function1;

public class KotlinHelper {
    public static <T> Function1<T, Unit> kbu(Consumer<T> consumer) {
        return (t) -> {
            consumer.accept(t);
            return Unit.INSTANCE;
        };
    }

    public static Function0<Unit> kbu(Runnable runnable) {
        return () -> {
            runnable.run();
            return Unit.INSTANCE;
        };
    }
}
