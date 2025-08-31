package lib.ntext;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class NTParameterWrapper<T> {
    private final NetworkTableEntry entry;
    protected T value;
    protected T prevValue;

    public NTParameterWrapper(String tableName, T defaultValue) {
        entry = NetworkTableInstance.getDefault().getEntry(tableName);
        entry.setDefaultValue(defaultValue);
        value = defaultValue;
        prevValue = null; // NOTE: design choice to make change happen on default

        NTParameterRegistry.registerWrapper(this);
    }

    public T getValue() {
        return value; // must be correct, check at annotation processor
    }

    public T getPreviousValue() {
        return prevValue;
    }

    public boolean hasChanged() {
        return !Objects.equals(prevValue, value);
    }

    public void onChange(Consumer<T> current) {
        NTParameterRegistry.registerOnChange(this, current);
    }

    public void onChange(BiConsumer<T, T> currentPrevious) {
        NTParameterRegistry.registerOnChange(this, currentPrevious);
    }

    @SuppressWarnings("unchecked")
    public void refresh() {
        prevValue = value;
        value = (T) entry.getValue().getValue();
    }
}
