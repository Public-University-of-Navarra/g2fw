## Qué

<!-- Qué cambia este PR, en una o dos frases. -->

Closes #

## Checklist

- [ ] La descripción lleva `Closes #<issue>`.
- [ ] Solo he cambiado mi carpeta `fw/lib/<nombre>/` (y, en el paso 4, mi hueco de `fw/src/main.cpp`).
- [ ] Las funciones coinciden exactamente con el contrato (`fw/README.md`): nombres, tipos y unidades.
- [ ] No hay `delay()` en `fw/lib`, salvo dentro de `<nombre>_begin()` y marcado con `// setup-only`.
- [ ] La salida serie sigue el contrato: `[<nombre>] key=value` y `[<nombre>] ERR code=<n>`.
- [ ] Compila: `pio run -e esp32` dentro de `fw/`.
