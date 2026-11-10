# Catálogo de actuadores MyActuator

Datos de los actuadores MyActuator que usa `geometry_opt/` para seleccionar motores, más sus dimensiones y
curvas características. Fuente: páginas de descarga del fabricante
([serie X](https://www.myactuator.com/downloads-xseries), [serie L](https://www.myactuator.com/downloads-lseries)),
consultadas el 2026-11-10. Las fichas originales están en [`fichas/`](fichas/).

## Contenido

| Archivo / carpeta | Qué es |
|---|---|
| `motors_MyActuator_Series_L_H_X.csv` | **Catálogo principal** (36 motores). Es el que lee `geometry_opt/catalog.py`. |
| `motors_MyActuator_Series_L_H.csv` | Subconjunto de las series L y H (mismas columnas). |
| `motors_MyActuator.xlsx` | Las mismas 36 filas en Excel. |
| `curvas/<motor>.csv` | Curva característica de 19 motores; [`curvas/indice.csv`](curvas/indice.csv) dice de dónde sale cada una. |
| `fichas/<motor>_ficha.pdf` | Ficha técnica (*Motor Parameter* / *Product information*) de los 29 motores de la web. |
| `fichas/<motor>_especificaciones.pdf` | Manual de especificaciones (14 págs.) de X4-10-L, X4-36-L, X6-50-L, X8-150 y X10-200. |
| `herramientas/` | Scripts usados para extraer las curvas (ver al final). |

Formato de los CSV del catálogo: separador `;`, coma decimal, UTF-8 con BOM, `N/A` cuando falta el dato.

## Columnas

Las 14 primeras columnas son las originales:

| Columna | Significado / nota |
|---|---|
| `motor_name` | `MyActuator <modelo> <reducción>` |
| `rated_voltage_V` | Tensión nominal |
| `rated_voltage_A` | **Corriente** nominal (A rms). El nombre es un error histórico; `catalog.py` la renombra a `rated_current_A`. |
| `nominal_torque_Nm`, `peak_torque_Nm` | Par nominal y de pico a la salida |
| `nominal_speed_rpm`, `max_speed_rpm`, `no_load_speed_rpm` | Velocidad nominal, máxima y sin carga a la salida |
| `max_current_A` | Corriente de fase de pico (A rms) |
| `speed_constant_rpm_per_V` | En la serie L es la constante de velocidad (rpm/V). En las filas X V4 contiene la **constante de fuerza contraelectromotriz (Vdc/krpm)** de la ficha: no es la misma magnitud. |
| `torque_constant_Nm_per_A` | Constante de par del módulo (N·m/A rms) |
| `motor_mass_kg`, `stall_torque_Nm` | Masa y par de bloqueo |
| `efficiency` | Ruta a un mapa de eficiencia de otro proyecto; esos archivos **no** están en este repo. |

Columnas añadidas el 2026-11-10:

| Columna | Significado |
|---|---|
| `diameter_mm`, `length_mm` | Diámetro exterior máximo y largo total del cuerpo (el "55×61" de la ficha del X4-36). |
| `radius_mm` | `diameter_mm / 2` |
| `radius_with_5mm_clearance_mm` | `radius_mm + 5` (holgura de 5 mm sobre el radio). |
| `dimensions_source` | `ficha (Dimensions)` si la ficha trae la fila *Dimensions*; `plano` si se leyó de las cotas del plano de instalación. |
| `characteristic_curve_file`, `characteristic_curve_source` | Archivo de la curva en `curvas/` y cómo se obtuvo (vacío si el catálogo no trae curva). |

Siete motores del CSV original no aparecen en las páginas de descarga X/L (X4-3, X8-20, X8-60 y la serie H):
quedan con dimensiones `N/A`.

## Cambios del 2026-11-10

- **Motores nuevos** (están en la web pero no estaban en el CSV): X4-10-L, X4-36-L, X6-50-L, X8-150, X10-200
  y X15-450. Para las versiones -L, X8-150 y X10-200 la ficha da la corriente nominal en A pico; se guardó en A rms
  (pico/√2, coincide con las versiones no-L) y como `max_current_A` se usó la corriente de fase de bloqueo (A rms).
- **`X8-102` → `X8-120`**: era un error de tipeo (los datos son los de la ficha del X8-120).
- Dimensiones de los 29 motores de la web y curvas características de 19.

> ⚠️ **Esto cambia los resultados de `geometry_opt`.** Con los motores nuevos el pipeline elige X6-50-L en vez de
> X6-60 en la rodilla delantera derecha y X4-10-L en vez de X4-10, y la masa final baja de 8.77 a 8.73 kg (v0218,
> PLA) y de 9.72 a 9.68 kg (v140, PLA). `geometry_opt/results_summary.md` **todavía tiene los resultados con el
> catálogo anterior**; regenéralo con `python -m geometry_opt.run_pipeline` (desde `simulacion/`) si quieres usar
> el catálogo nuevo.

> La web de MyActuator marca como *Discontinued* los X4-10 y X4-36 de la serie V4, el X10-100 y los X6-7, X6-40
> y X8-25 (serie V2). Las versiones -L (V4.1) son las vigentes. Conviene confirmarlo con el fabricante antes de
> comprar.

## Curvas características (`curvas/`)

Columnas: `torque_Nm, speed_rpm, output_power_W, voltage_V, current_A, input_power_W, efficiency_pct`. Velocidad,
par y potencia son a la salida del actuador. Cada fila es un punto del ensayo de carga del fabricante, en el orden
de la ficha (de vacío a carga máxima).

| Fuente (`characteristic_curve_source`) | Motores | Precisión |
|---|---|---|
| `tabla (texto PDF)` | L-7015, L-7025, L-9015, L-9025, X6-7, X6-8, X6-40, X8-25, X10-40, X10-100 | Exacta (texto del PDF). En la serie L el par venía en mN·m y se pasó a N·m. |
| `tabla (imagen, transcrita)` | X4-10-L, X4-36-L, X6-50-L, X8-150, X10-200 | Tabla en imagen del manual de especificaciones, transcrita a mano y verificada contra OCR y contra Pout = T·ω, Pin = U·I y Eff = Pout/Pin. |
| `grafica digitalizada` | L-4005, L-4010, L-4015, L-5010 | La ficha solo trae la gráfica. Digitalizada por color en 21 velocidades; la potencia leída coincide con T·ω con error mediano de 0.5–1.4 % (máx. < 5 %). No traen `voltage_V` ni `input_power_W` porque la ficha no dice a qué tensión se hizo el ensayo. |

Los datos se conservan tal como los da el fabricante, incluidas sus pequeñas inconsistencias: en algunos motores
la columna Eff es ~2 puntos menor que Pout/Pin, y en X8-150 las dos últimas filas no cumplen Pin = U·I.

**Sin curva:** L-5005 y L-5015 (la gráfica es ilegible y la del L-5015 mezcla dos variantes, 10T y 35T), y las
fichas V4 de X2-7, X4-10, X4-36, X6-60, X8-120, X8-32, X12-320 y X15-450, que solo traen una tabla de par de
bloqueo (*Stall Torque Data*). Para X4-10 y X4-36 sirven como referencia las curvas de sus versiones -L.

## Herramientas (`herramientas/`)

Scripts usados para extraer las curvas, por si hay que repetirlo con fichas nuevas (necesitan `poppler-utils`,
`poppler-data` y, para la digitalización, `numpy` y `pillow`):

- `curve_from_pdf.py <ficha.pdf>`: reconstruye la tabla de la curva desde el texto del PDF (fichas con tabla en texto).
- `ocr_curve.py`, `compare.py`: OCR de una tabla en imagen y comparación con una transcripción manual + chequeos físicos.
- `digitize_L.py`, `fit_L.py`, `build_L.py`: digitalización por color de las gráficas de la serie L (los máximos de
  cada eje se leen a mano de la imagen y se pasan como argumentos; ver el docstring de cada script).
