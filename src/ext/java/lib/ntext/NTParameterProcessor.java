package lib.ntext;

import com.google.auto.service.AutoService;

import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.lang.model.util.ElementFilter;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;
import java.io.IOException;
import java.io.Writer;
import java.util.*;

@SupportedAnnotationTypes({"lib.ntext.NTParameter"})
@SupportedSourceVersion(SourceVersion.RELEASE_17)
@SupportedOptions("DEBUG")
@AutoService(Processor.class)
public class NTParameterProcessor extends AbstractProcessor {
    private static final Map<String, String> NT_TYPES_TABLE = Map.ofEntries(
            // Booleans
            Map.entry("boolean", "Boolean"), Map.entry("java.lang.Boolean", "Boolean"),
            Map.entry("boolean[]", "Boolean[]"), Map.entry("java.lang.Boolean[]", "Boolean[]"),

            // Integers
            Map.entry("int", "Integer"), Map.entry("java.lang.Integer", "Integer"), Map.entry("int[]", "Integer[]"),
            Map.entry("java.lang.Integer[]", "Integer[]"),

            Map.entry("long", "Long"), Map.entry("java.lang.Long", "Long"), Map.entry("long[]", "Long[]"),
            Map.entry("java.lang.Long[]", "Long[]"),

            // Floats
            Map.entry("float", "Float"), Map.entry("java.lang.Float", "Float"), Map.entry("float[]", "Float[]"),
            Map.entry("java.lang.Float[]", "Float[]"),

            // Doubles
            Map.entry("double", "Double"), Map.entry("java.lang.Double", "Double"), Map.entry("double[]", "Double[]"),
            Map.entry("java.lang.Double[]", "Double[]"),

            // Strings
            Map.entry("java.lang.String", "String"), Map.entry("java.lang.String[]", "String[]"),

            // Raw bytes
            Map.entry("byte[]", "Byte[]")
    );

    /**
     * Main processing method for dealing with all the annotations.
     *
     * @param annotations set all annotated classes.
     * @param roundEnv    processing env for the processor.
     * @return if the processor runs successfully.
     */
    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        // Group annotated elements by their containing class
        Map<TypeElement, List<VariableElement>> classToFields = new HashMap<>();
        Set<TypeElement> annotatedClasses = new HashSet<>();

        for (Element annotated : roundEnv.getElementsAnnotatedWith(NTParameter.class)) {
            if (annotated.getKind() == ElementKind.CLASS) {
                // Class-level annotation
                TypeElement rootClass = (TypeElement) annotated;
                annotatedClasses.add(rootClass);
                validateFields(rootClass);
                generateWrapper(rootClass);
            } else if (annotated.getKind() == ElementKind.FIELD) {
                // Field-level annotation
                VariableElement field = (VariableElement) annotated;
                TypeElement containingClass = (TypeElement) field.getEnclosingElement();

                // Validate the field
                if (!validateSingleField(field)) {
                    continue;
                }


                // Group fields by their containing class
                classToFields.computeIfAbsent(containingClass, k -> new ArrayList<>()).add(field);
            } else {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR, "@lib.ntext.NTParameter can only be applied to classes or fields.",
                        annotated
                );
            }
        }

        // Process field-level annotations grouped by class
        for (Map.Entry<TypeElement, List<VariableElement>> entry : classToFields.entrySet()) {
            TypeElement containingClass = entry.getKey();
            List<VariableElement> fields = entry.getValue();

            // Skip if this class was already processed as a class-level annotation
            if (annotatedClasses.contains(containingClass)) {
                continue;
            }

            // Generate field wrappers
            generateFieldWrapper(containingClass, fields);
        }

        return true;
    }

    /**
     * Validate a single field for field-level annotations.
     */
    private boolean validateSingleField(VariableElement field) {
        String className = field.getEnclosingElement().getSimpleName().toString();

        if (!field.getModifiers().contains(Modifier.STATIC)) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR,
                    "Field '" + field.getSimpleName() + "' in class '" + className + "' must be declared static.",
                    field
            );
            return false;
        }

        String typeName = field.asType().toString();
        if (!NT_TYPES_TABLE.containsKey(typeName)) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR,
                    "Invalid field type '" + typeName + "' in class '" + className + "'.",
                    field
            );
            return false;
        }

        return true;
    }

    /**
     * Recursive method to check all the fields of a class.
     *
     * @param classElement root class.
     */
    private void validateFields(TypeElement classElement) {
        String className = classElement.getQualifiedName().toString();
        List<VariableElement> fields = ElementFilter.fieldsIn(classElement.getEnclosedElements());
        for (VariableElement field : fields) {
            if (!field.getModifiers().contains(Modifier.STATIC)) {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR,
                        "Field '" + field.getSimpleName() + "' in class '" + className + "' must be declared static.",
                        field
                );
                continue;
            }

            String typeName = field.asType().toString();
            if (!NT_TYPES_TABLE.containsKey(typeName)) {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR, "Invalid field type '" + typeName + "' in class '" + className + "'.",
                        field
                );
                throw new FieldTypeError("Invalid field type '" + typeName + "' in class '" + className + "'.");
            }
        }

        for (Element enclosed : classElement.getEnclosedElements()) {
            if (enclosed.getKind() == ElementKind.CLASS && enclosed.getModifiers().contains(Modifier.STATIC)) {
                validateFields((TypeElement) enclosed);
            } else if (enclosed.getKind() == ElementKind.CLASS) {
                processingEnv.getMessager().printMessage(
                        Diagnostic.Kind.ERROR,
                        "Nested class '" + enclosed.getSimpleName() + "' in '" + className + "' must be declared static.",
                        enclosed
                );
                throw new FieldTypeError("Nested class '" + enclosed.getSimpleName() + "' in '" + className + "'.");
            }
        }
    }

    /**
     * Generate wrapper for field-level annotations.
     */
    /**
     * Generate wrapper for field-level annotations.
     */
    private void generateFieldWrapper(TypeElement containingClass, List<VariableElement> annotatedFields) {
        // Use the first field's annotation for table name, or derive from class name
        String tableName = null;
        for (VariableElement field : annotatedFields) {
            NTParameter annotation = field.getAnnotation(NTParameter.class);
            if (annotation != null && !annotation.tableName().isEmpty()) {
                tableName = annotation.tableName();
                break;
            }
        }

        if (tableName == null) tableName = containingClass.getSimpleName().toString();

        String pkgName = processingEnv.getElementUtils().getPackageOf(containingClass).toString();
        String className = containingClass.getSimpleName() + "NT";

        // start builder, write header
        StringBuilder builder = new StringBuilder();
        builder.append("package ").append(pkgName).append(";\n\n")
                .append("import edu.wpi.first.networktables.NetworkTableEntry;\n")
                .append("import edu.wpi.first.networktables.NetworkTableInstance;\n\n")
                .append("import lib.ntext.NTParameterWrapper;\n\n")
                .append("public class ").append(className).append(" {\n");

        // Track field names for isAnyChanged()
        List<String> fieldNames = new ArrayList<>();

        // Process only the annotated fields
        for (VariableElement field : annotatedFields) {
            String fieldName = field.getSimpleName().toString();
            String typeName = NT_TYPES_TABLE.get(field.asType().toString());
            Object defaultValue = field.getConstantValue();
            String defaultLiteral = getDefaultLiteral(defaultValue, typeName);

            builder.append("  public static final NTParameterWrapper<")
                    .append(typeName).append("> ").append(fieldName)
                    .append(" = new NTParameterWrapper<>(")
                    .append("\"").append(tableName).append("/").append(fieldName).append("\", ")
                    .append(defaultLiteral).append(");\n");

            fieldNames.add(fieldName);
        }

        // Append isAnyChanged() if any fields exist
        if (!fieldNames.isEmpty()) {
            builder.append("\n  public static boolean isAnyChanged() {\n")
                    .append("    return ")
                    .append(String.join(" || ", fieldNames.stream().map(f -> f + ".hasChanged()").toList()))
                    .append(";\n  }\n");
        }

        // end, write as a generated java file
        builder.append("}\n");

        try {
            JavaFileObject file = processingEnv.getFiler().createSourceFile(pkgName + "." + className, containingClass);
            try (Writer writer = file.openWriter()) {
                writer.write(builder.toString());
            }
        } catch (IOException e) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR, "Codegen failed: " + e.getMessage(), containingClass);
        }
    }


    /**
     * Entry method for generating corresponding network table method for the class.
     *
     * @param rootClass root class.
     */
    private void generateWrapper(TypeElement rootClass) {
        String tableName = rootClass.getAnnotation(NTParameter.class).tableName();
        String pkgName = processingEnv.getElementUtils().getPackageOf(rootClass).toString();
        String className = rootClass.getSimpleName() + "NT";

        // start builder, write header
        StringBuilder builder = new StringBuilder();
        builder.append("package ").append(pkgName).append(";\n\n").append(
                "import edu.wpi.first.networktables.NetworkTableEntry;\n").append(
                "import edu.wpi.first.networktables.NetworkTableInstance;\n\n").append(
                "import lib.ntext.NTParameterWrapper;\n\n").append("public class ").append(className).append(" {\n");

        // do recursive adding
        buildClassContent(rootClass, tableName, builder, "  ", "");

        // end, write as a generated java file
        builder.append("}\n");
        try {
            JavaFileObject file = processingEnv.getFiler().createSourceFile(pkgName + "." + className, rootClass);
            try (Writer writer = file.openWriter()) {
                writer.write(builder.toString());
            }
        } catch (IOException e) {
            processingEnv.getMessager().printMessage(
                    Diagnostic.Kind.ERROR, "Codegen failed: " + e.getMessage(), rootClass);
        }
    }

    /**
     * Get default literal value for a field.
     */
    private String getDefaultLiteral(Object defaultValue, String typeName) {
        if (defaultValue != null) {
            if (defaultValue instanceof String s) {
                return "\"" + s.replace("\"", "\\\"") + "\"";
            } else if (defaultValue instanceof Character c) {
                return "'" + c + "'";
            } else if (defaultValue instanceof Long) {
                return defaultValue + "L";
            } else if (defaultValue instanceof Float) {
                return defaultValue + "f";
            } else if (defaultValue instanceof Double) {
                return defaultValue + "d";
            } else {
                return defaultValue.toString();
            }
        }

        // Fallbacks by type
        switch (typeName) {
            case "java.lang.String":
                return "";
            case "boolean":
            case "java.lang.Boolean":
                return "false";
            case "int":
            case "java.lang.Integer":
            case "long":
            case "java.lang.Long":
                return "0";
            case "float":
            case "java.lang.Float":
                return "0.0f";
            case "double":
            case "java.lang.Double":
                return "0.0";
            case "byte[]":
                return "new byte[0]";
            default:
                if (typeName.endsWith("[]")) {
                    return "new " + typeName.replace("[]", "") + "[0]";
                } else {
                    return "null";
                }
        }
    }

    /**
     * Recursive method for generating NTParameterWrapper fields and isAnyChanged() methods.
     *
     * @param classElement current class
     * @param tablePath    base NetworkTable path
     * @param builder      output builder
     * @param indent       indentation for current scope
     * @param prefix       NetworkTable key prefix for nested fields (e.g., "Outer/Inner/")
     */
    private void buildClassContent(TypeElement classElement, String tablePath, StringBuilder builder,
                                   String indent, String prefix) {
        List<VariableElement> fields = ElementFilter.fieldsIn(classElement.getEnclosedElements());
        List<String> fieldNames = new ArrayList<>();

        for (VariableElement field : fields) {
            String fieldName = field.getSimpleName().toString();
            String typeName = NT_TYPES_TABLE.get(field.asType().toString());
            Object defaultValue = field.getConstantValue();
            String defaultLiteral = getDefaultLiteral(defaultValue, typeName);
            String fullKey = tablePath + "/" + prefix + fieldName;

            builder.append(indent).append("public static final NTParameterWrapper<").append(typeName).append("> ")
                    .append(fieldName).append(" = new NTParameterWrapper<>(")
                    .append("\"").append(fullKey).append("\", ").append(defaultLiteral).append(");\n");

            fieldNames.add(fieldName);
        }

        if (!fieldNames.isEmpty()) {
            builder.append("\n").append(indent).append("public static boolean isAnyChanged() {\n")
                    .append(indent).append("  return ")
                    .append(String.join(" || ", fieldNames.stream().map(f -> f + ".hasChanged()").toList()))
                    .append(";\n").append(indent).append("}\n\n");
        }

        for (Element enclosed : classElement.getEnclosedElements()) {
            if (enclosed.getKind() == ElementKind.CLASS && enclosed instanceof TypeElement nested) {
                String nestedName = nested.getSimpleName().toString();
                builder.append(indent).append("public static class ").append(nestedName).append(" {\n");
                buildClassContent(nested, tablePath, builder, indent + "  ", prefix + nestedName + "/");
                builder.append(indent).append("}\n");
            }
        }
    }

    /**
     * Build content for specific fields (used for field-level annotations).
     */
    private void buildFieldsContent(
            List<VariableElement> fields, String tableName, StringBuilder builder, String indent) {
        for (VariableElement field : fields) {
            String fieldName = field.getSimpleName().toString();
            String typeName = NT_TYPES_TABLE.get(field.asType().toString());
            Object defaultValue = field.getConstantValue();
            String defaultLiteral = getDefaultLiteral(defaultValue, typeName);

            builder.append(indent).append("public static final NTParameterWrapper<").append(typeName).append(
                    "> ").append(fieldName).append(" = new NTParameterWrapper<>(").append("\"").append(
                    tableName).append("/").append(fieldName).append("\", ").append(defaultLiteral).append(");\n");

        }
    }

    public static class FieldTypeError extends Error {
        public FieldTypeError(String message) {
            super(message);
        }
    }
}