package lib.ntext;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class NTParameterRegistry {
    private static final List<NTParameterWrapper<?>> wrappers = new ArrayList<>();
    private static final List<Pair<NTParameterWrapper<?>, Consumer<Object>>> onchangeSiConsumers = new ArrayList<>();
    private static final List<Pair<NTParameterWrapper<?>, BiConsumer<Object, Object>>> onchangeBiConsumers = new ArrayList<>();

    protected static void registerWrapper(NTParameterWrapper<?> wrapper) {
        wrappers.add(wrapper);
    }

    @SuppressWarnings("unchecked")
    protected static void registerOnChange(NTParameterWrapper<?> wrapper, BiConsumer<?, ?> functor) {
        BiConsumer<Object, Object> castedFunctor = (BiConsumer<Object, Object>) functor;
        onchangeBiConsumers.add(Pair.of(wrapper, castedFunctor));
    }

    @SuppressWarnings("unchecked")
    protected static void registerOnChange(NTParameterWrapper<?> wrapper, Consumer<?> functor) {
        Consumer<Object> castedFunctor = (Consumer<Object>) functor;
        onchangeSiConsumers.add(Pair.of(wrapper, castedFunctor));
    }

    public static void refresh() {
        onchangeSiConsumers.forEach(pair -> {
            if (pair.getFirst().hasChanged())
                pair.getSecond().accept(pair.getFirst().getValue());
        });
        onchangeBiConsumers.forEach(pair -> {
            if (pair.getFirst().hasChanged())
                pair.getSecond().accept(pair.getFirst().getValue(), pair.getFirst().getPreviousValue());
        });
        wrappers.forEach(NTParameterWrapper::refresh);
    }
}
