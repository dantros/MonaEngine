#!/usr/bin/env python3
# -*- coding: utf-8 -*-

MD_PATH = r'g:/projects/mona_engine_/MonaEngine/Memorias/Memoria_Agustin_Matthey/Memoria_Agustin_Matthey.md'

with open(MD_PATH, 'r', encoding='utf-8') as f:
    text = f.read()

def r(old, new, t):
    if old not in t:
        print(f"WARNING: not found: {repr(old[:60])}")
    return t.replace(old, new)

# ── STEP 1: CID glyph artifacts ──────────────────────────────────────────────
text = text.replace('(cid:88)', '∑')
text = text.replace('(cid:89)', '∏')
text = text.replace('(cid:80)', '∑')
text = text.replace('(cid:34)', '[')
text = text.replace('(cid:35)', ']')
text = text.replace('(cid:104)', '[')
text = text.replace('(cid:105)', ']')

# ── STEP 2: Section 2.4.2 – FK equation ──────────────────────────────────────
text = r(
'''Considérese una cadena articulada con articulaciones j ,...,j ,...,j , cada una con una trans-
1 i n
formación M asociada , donde la articulacíon j es la base de la cadena y j es el end-effector.
i 1 n
Para cada transformación M , variable en el tiempo, se cumple que M (t) = T (t)R (t)S (t),
i i i i i
donde T ,R ,S son las subtransformaciones de traslacíon, rotación y escalamiento respecti-
i i i
vamente. T y S se mantienen fijas para mantener las distancias relativas, y R es variable
i i i
en el tiempo, por lo que M (t) = T R (t)S . La posicíon del end-effector en el espacio padre
i i i i
de la base de la cadena se calcula como:
[ ]
n
∏
Pos (t) = T R (t)S p\u20d7 (2.2)
ee i i i
i=1
El vector p\u20d7 = {0,0,0,1}, es la posición del end-effector en su propio espacio local. Dado
que el end-effector se encuentra en el origen de su propio sistema de referencia, aplicarle
una rotacíon a su posicíon local no genera en ella ningún cambio. Para cualquier matriz
de rotacíon R, se cumple que p\u20d7 = Rp\u20d7. Esto implica que modificar la rotación asociada al
end-effector, no altera el resultado de la ecuación 2.2.''',
'''Considérese una cadena articulada con articulaciones $j_1, \\ldots, j_i, \\ldots, j_n$, cada una con una transformación $M_i$ asociada, donde la articulación $j_1$ es la base de la cadena y $j_n$ es el end-effector. Para cada transformación $M_i$, variable en el tiempo, se cumple que $M_i(t) = T_i(t) R_i(t) S_i(t)$, donde $T_i, R_i, S_i$ son las subtransformaciones de traslacíon, rotación y escalamiento respectivamente. $T_i$ y $S_i$ se mantienen fijas para mantener las distancias relativas, y $R_i$ es variable en el tiempo, por lo que $M_i(t) = T_i R_i(t) S_i$. La posicíon del end-effector en el espacio padre de la base de la cadena se calcula como:

$$\\text{Pos}_{ee}(t) = \\prod_{i=1}^n T_i R_i(t) S_i \\, \\vec{p} \\tag{2.2}$$

El vector $\\vec{p} = \\{0,0,0,1\\}$, es la posición del end-effector en su propio espacio local. Dado que el end-effector se encuentra en el origen de su propio sistema de referencia, aplicarle una rotacíon a su posicíon local no genera en ella ningún cambio. Para cualquier matriz de rotacíon $R$, se cumple que $\\vec{p} = R\\vec{p}$. Esto implica que modificar la rotación asociada al end-effector, no altera el resultado de la ecuación 2.2.''',
text)

# ── STEP 3: Section 2.4.3 – IK problem formulation ───────────────────────────
text = r(
'''\u20d7
Defínase \u03b8, como el vector que contiene un ángulo \u03b8 por cada articulación j de la cadena
i i
K, que indica cuanto ha rotado j desde su estado de reposo en un determinado plano de

\u20d7
rotación. El vector \u03b8 permite determinar completamente el estado de la cadena, teníendose
como base las distancias fijas entre las articulaciones. Con cinemática directa (FK) puede
tomarse esta informacíon rotacional, y obtenerse la posición del end-effector en el espacio
de la base de la cadena. Se puede expresar entonces la posición del end-effector en función
\u20d7
de las rotaciones de las articulaciones como \u20d7s = f(\u03b8), usando FK. Resolver el problema de
IK consiste en obtener la función inversa, tal que
f\u22121(\u20d7s)
= \u03b8
\u20d7
.
f\u22121
es una función altamente''',
'''Defínase $\\vec{\\theta}$, como el vector que contiene un ángulo $\\theta_i$ por cada articulación $j_i$ de la cadena K, que indica cuanto ha rotado $j$ desde su estado de reposo en un determinado plano de rotación. El vector $\\vec{\\theta}$ permite determinar completamente el estado de la cadena, teníendose como base las distancias fijas entre las articulaciones. Con cinemática directa (FK) puede tomarse esta informacíon rotacional, y obtenerse la posición del end-effector en el espacio de la base de la cadena. Se puede expresar entonces la posición del end-effector en función de las rotaciones de las articulaciones como $\\vec{s} = f(\\vec{\\theta})$, usando FK. Resolver el problema de IK consiste en obtener la función inversa, tal que

$$f^{-1}(\\vec{s}) = \\vec{\\theta}$$

$f^{-1}$ es una función altamente''',
text)

# ── STEP 4: Section 2.5 – LIC interpolation formula ──────────────────────────
text = r(
'''Una curva linealmente interpolada es en realidad una spline lineal [21]. Si una LIC l posee k puntos p , con sus

respectivos valores del parámetro t, t ,...,t ,...,t , se genera un intervalo de tiempo [t ,t ] para
1 i k 1 k
l. Es posible obtener un punto de la curva para cualquier instante de tiempo
t\u02c6perteneciente
a
ese intervalo, simulando continuidad. Si se da que \u2203i
t\u02c6=
t , entonces obtenemos directamente

el punto asociado a t y
l(t\u02c6)
= p . En otro caso, buscamos el subintervalo más pequeño que
i i
contenga a
t\u02c6,
tal que t <
t\u02c6
< t e interpolamos linealmente entre los puntos p y p
i i+1 i i+1
utilizando la relacíon entre t ,
t\u02c6
y t como referencia, obteníendose que:
i i+1
t\u02c6\u2212
t
l(t\u02c6)
= p + (p \u2212 p )

i i+1 i
t \u2212 t
i+1 i''',
'''Una curva linealmente interpolada es en realidad una spline lineal [21]. Si una LIC $l$ posee $k$ puntos $p_i$, con sus respectivos valores del parámetro $t$, $t_1, \\ldots, t_i, \\ldots, t_k$, se genera un intervalo de tiempo $[t_1, t_k]$ para $l$. Es posible obtener un punto de la curva para cualquier instante de tiempo $\\hat{t}$ perteneciente a ese intervalo, simulando continuidad. Si se da que $\\exists\\, i: \\hat{t} = t_i$, entonces obtenemos directamente el punto asociado a $t_i$ y $l(\\hat{t}) = p_i$. En otro caso, buscamos el subintervalo más pequeño que contenga a $\\hat{t}$, tal que $t_i < \\hat{t} < t_{i+1}$ e interpolamos linealmente entre los puntos $p_i$ y $p_{i+1}$ utilizando la relacíon entre $t_i$, $\\hat{t}$ y $t_{i+1}$ como referencia, obteníendose que:

$$l(\\hat{t}) = p_i + (p_{i+1} - p_i)\\,\\frac{\\hat{t} - t_i}{t_{i+1} - t_i}$$''',
text)

# ── STEP 5: Section 2.6 – Gradient descent ───────────────────────────────────
text = r(
'''El descenso de gradiente [5] es una técnica de optimizacíon iterativa, que permite en-
contrar mínimos locales en una función escalar, multivariable, y diferenciable. Si se tiene
una función F, que cumple con las características anteriores, se quiere encontrar un vector
\u20d7x = {x ,...,x ,...,x } tal que F(\u20d7x) sea un mínimo local. En cada iteración debe calcularse el
1 i n
gradiente de F evaluado en \u20d7x:

\u2202F(\u20d7x)
\u2202x1

...


\u2207F(\u20d7x) =
\u2202F(\u20d7x)


\u2202xi


...

\u2202F(\u20d7x)
\u2202xn
El gradiente de F evaluado en \u20d7x es un vector, e indica la dirección de máximo crecimiento
de F. Teniendo calculado el gradiente, se actualiza el valor de \u20d7x:
\u20d7x := \u20d7x \u2212 \u03bb\u2207F(\u20d7x)''',
'''El descenso de gradiente [5] es una técnica de optimizacíon iterativa, que permite encontrar mínimos locales en una función escalar, multivariable, y diferenciable. Si se tiene una función $F$, que cumple con las características anteriores, se quiere encontrar un vector $\\vec{x} = \\{x_1, \\ldots, x_i, \\ldots, x_n\\}$ tal que $F(\\vec{x})$ sea un mínimo local. En cada iteración debe calcularse el gradiente de $F$ evaluado en $\\vec{x}$:

$$\\nabla F(\\vec{x}) = \\begin{pmatrix} \\dfrac{\\partial F(\\vec{x})}{\\partial x_1} \\\\ \\vdots \\\\ \\dfrac{\\partial F(\\vec{x})}{\\partial x_i} \\\\ \\vdots \\\\ \\dfrac{\\partial F(\\vec{x})}{\\partial x_n} \\end{pmatrix}$$

El gradiente de $F$ evaluado en $\\vec{x}$ es un vector, e indica la dirección de máximo crecimiento de $F$. Teniendo calculado el gradiente, se actualiza el valor de $\\vec{x}$:

$$\\vec{x} := \\vec{x} - \\lambda\\nabla F(\\vec{x})$$''',
text)

# ── STEP 6: Section 3.1.2 – Jacobian IK methods ──────────────────────────────
text = r(
'''Recordando el planteamiento original de IK (2.4.3), se busca solucionar la ecuación
f\u22121(\u20d7s)
=
\u20d7 \u20d7
\u03b8. Generalizando, \u03b8 es el vector de rotaciones, y \u20d7s es un vector con las posiciones de un
número arbitrario end-effectors. Si se considera \u20d7s como las posiciones actuales, y
\u20d7t
como las
posiciones objetivo, se puede definir un vector de error \u20d7e, tal que \u20d7e =
\u20d7t\u2212\u20d7s.
Se quiere usar los
métodos numéricos para modificar \u20d7s, acercándolo a
\u20d7t
lo más posible y minimizando el error
\u20d7e.
Métodos basados en la inversíon del Jacobiano
Se puede escribir el jacobiano del vector \u03b8 como J(\u03b8
\u20d7
) =
(\u2202si
) . Las entradas de la
ij
\u2202\u03b8j
ij
matriz J pueden calcularse como
(\u2202\u20d7si
) = v \u00d7 (\u20d7s \u2212 p\u20d7 ), donde v\u20d7 es el vector unitario
\u2202\u03b8j
ij j i j j
que apunta en la direccíon del eje actual de rotacíon de la j-ésima articulación, y p\u20d7 es
j
su posición actual. Además la derivada de \u20d7s con respecto al tiempo puede escribirse como
\u02d9
\u02d9
\u20d7 \u20d7
\u20d7s = J(\u03b8)\u03b8. Con esto, una pequeña variación del vector \u20d7s puede aproximarse como \u2206\u20d7s \u2248
\u20d7 \u20d7
J\u2206\u03b8. La idea es elegir un vector \u2206\u03b8 que haga que \u2206\u20d7s se aproxime lo más posible a \u20d7e,
logrando así un acercamiento a la posicíon objetivo con ese pequeño cambio. Por último,
\u20d7
se puede obtener la variación de \u03b8 requerida si se calcula el inverso del jacobiano J, ya que
\u2206\u03b8
\u20d7
=
J\u22121\u20d7e.
Para resolver esta ecuacíon en general se buscan alternativas que eviten tener''',
'''Recordando el planteamiento original de IK (2.4.3), se busca solucionar la ecuación $f^{-1}(\\vec{s}) = \\vec{\\theta}$. Generalizando, $\\vec{\\theta}$ es el vector de rotaciones, y $\\vec{s}$ es un vector con las posiciones de un número arbitrario end-effectors. Si se considera $\\vec{s}$ como las posiciones actuales, y $\\vec{t}$ como las posiciones objetivo, se puede definir un vector de error $\\vec{e}$, tal que $\\vec{e} = \\vec{t} - \\vec{s}$. Se quiere usar los métodos numéricos para modificar $\\vec{s}$, acercándolo a $\\vec{t}$ lo más posible y minimizando el error $\\vec{e}$.

**Métodos basados en la inversíon del Jacobiano**

Se puede escribir el jacobiano del vector $\\vec{\\theta}$ como $J(\\vec{\\theta}) = \\left(\\dfrac{\\partial s_i}{\\partial \\theta_j}\\right)_{ij}$. Las entradas de la matriz $J$ pueden calcularse como $\\left(\\dfrac{\\partial \\vec{s}_i}{\\partial \\theta_j}\\right)_{ij} = \\vec{v}_j \\times (\\vec{s}_i - \\vec{p}_j)$, donde $\\vec{v}_j$ es el vector unitario que apunta en la dirección del eje actual de rotación de la $j$-ésima articulación, y $\\vec{p}_j$ es su posición actual. Además la derivada de $\\vec{s}$ con respecto al tiempo puede escribirse como $\\dot{\\vec{s}} = J(\\vec{\\theta})\\dot{\\vec{\\theta}}$. Con esto, una pequeña variación del vector $\\vec{s}$ puede aproximarse como $\\Delta\\vec{s} \\approx J\\Delta\\vec{\\theta}$. La idea es elegir un vector $\\Delta\\vec{\\theta}$ que haga que $\\Delta\\vec{s}$ se aproxime lo más posible a $\\vec{e}$, logrando así un acercamiento a la posicíon objetivo con ese pequeño cambio. Por último, se puede obtener la variación de $\\vec{\\theta}$ requerida si se calcula el inverso del jacobiano $J$, ya que $\\Delta\\vec{\\theta} = J^{-1}\\vec{e}$. Para resolver esta ecuacíon en general se buscan alternativas que eviten tener''',
text)

text = r(
'''i Transpuesta del jacobiano: Se modifica la ecuacíon para reemplazar
J\u22121,
quedando como
\u2206\u03b8
\u20d7
=
\u03b1JT\u20d7e,
donde
JT
es la transpuesta de J y \u03b1 es un escalar que puede calcularse como
\u03b1 =
\u20d7e\u00b7JJT\u20d7e
, siendo \u00b7 el producto punto. Esta solución suele requerir muchas iteraciones
JJT\u20d7e\u00b7JJT\u20d7e
\u20d7
(cálculos consecutivos de valores pequeños \u2206\u03b8) para acercarse de forma aceptable al
objetivo
\u20d7t,
y es común que genere poses poco creíbles y movimientos faltos de fluidez.
Estos problemas se dan principalmente cuando el objetivo esta muy lejos de la posicíon
inicial. Para evitar problemas es tambíen ideal que el valor de \u03b1 sea pequeño.
ii Pseudo-inversa del jacobiano: En este caso la ecuacíon es \u2206\u03b8
\u20d7
=
\u03b1Jpi\u20d7e,
donde
Jpi
es la
pseudo-inversa del jacobiano o inversa Moore-Penrose. La pseudo-inversa puede calcularse
como
Jpi
=
JT(JJT)\u22121.
Esta solución, en caso de estar cerca de una singularidad es
especialmente propensa a generar cambios drásticos en los ángulos de las articulaciones
aunque el cambio en la posicíon del end-effector sea muy pequeño.''',
'''i) **Transpuesta del jacobiano:** Se modifica la ecuacíon para reemplazar $J^{-1}$, quedando como $\\Delta\\vec{\\theta} = \\alpha J^T\\vec{e}$, donde $J^T$ es la transpuesta de $J$ y $\\alpha$ es un escalar que puede calcularse como

$$\\alpha = \\frac{\\vec{e} \\cdot JJ^T\\vec{e}}{JJ^T\\vec{e} \\cdot JJ^T\\vec{e}}$$

siendo $\\cdot$ el producto punto. Esta solución suele requerir muchas iteraciones (cálculos consecutivos de valores pequeños $\\Delta\\vec{\\theta}$) para acercarse de forma aceptable al objetivo $\\vec{t}$, y es común que genere poses poco creíbles y movimientos faltos de fluidez. Estos problemas se dan principalmente cuando el objetivo esta muy lejos de la posicíon inicial. Para evitar problemas es tambíen ideal que el valor de $\\alpha$ sea pequeño.

ii) **Pseudo-inversa del jacobiano:** En este caso la ecuacíon es $\\Delta\\vec{\\theta} = \\alpha J_{pi}\\vec{e}$, donde $J_{pi}$ es la pseudo-inversa del jacobiano o inversa Moore-Penrose. La pseudo-inversa puede calcularse como $J_{pi} = J^T(JJ^T)^{-1}$. Esta solución, en caso de estar cerca de una singularidad es especialmente propensa a generar cambios drásticos en los ángulos de las articulaciones aunque el cambio en la posicíon del end-effector sea muy pequeño.''',
text)

text = r(
'''ejemplo, usarse la siguiente función:
F(\u03b8
\u20d7
) = ||\u20d7s(\u03b8
\u20d7
)
\u2212\u20d7t||2
(3.1)''',
'''ejemplo, usarse la siguiente función:

$$F(\\vec{\\theta}) = ||\\vec{s}(\\vec{\\theta}) - \\vec{t}||^2 \\tag{3.1}$$''',
text)

# ── STEP 7: Section 4.5 – Gradient descent implementation ────────────────────
text = r(
'''Considérese que se quiere
aplicar la técnica a las funciones f ,...,f ,...,f en conjunto, ya que cada una de ellas tiene
1 i n
un significado en el contexto del problema a resolver. Estas funciones dependen del mismo
vector de variables \u20d7x = {x ,...,x ,...,x }. Se construye una función total F(\u20d7x) =\u2211n
f (\u20d7x),
1 k m
i=1

a la que finalmente se aplica el descenso de gradiente. Cada subfunción (o término) f , se

\u20d7
encapsula en un FunctionTerm que permite calcular su valor f (\u02c6x), y su derivada parcial

\u2202fi(\u20d7\u02c6x).
Cada término tiene tambíen asignado un peso w , que determina su importancia para
\u2202\u02c6x

k
el cálculo del valor final.


La clase GradientDescent utiliza la funcíon de derivada parcial en cada término para
construir el vector gradiente:

\u2202fi(\u20d7x)
\u2202x1
 ...
\u2211
n

\u2207F(\u20d7x) = w
\u2202fi(\u20d7x)
i
\u2202x


k


i=1
...

\u2202fi(\u20d7x)
\u2202xm
Recordando lo expuesto en 2.6, el vector \u20d7x que minimiza localmente la función F se actualiza
como sigue:
\u20d7x := \u20d7x \u2212 \u03bb\u2207F(\u20d7x)
Esta es la forma regular de actualizar el gradiente, pero existen muchas variantes. La clase
GradientDescent, además del método regular, puede utilizar la técnica de descenso de gra-
diente con momentum, en que los cálculos de iteraciones pasadas tienen un peso en el cálculo
del valor actual del gradiente. El valor almacenado del gradiente, \u2207F(\u20d7x) , se actualiza de
saved
la siguiente manera utilizando momentum:
\u2207F(\u20d7x) := \u03b1\u2207F(\u20d7x) + (1 \u2212 \u03b1)\u2207F(\u20d7x)
saved saved
\u03b1 es el factor que indica cúanto pesa el gradiente histórico \u2207F(\u20d7x) , en relación al gradiente
saved
calculado en la iteracíon actual \u2207F(\u20d7x). En este caso particular, se utiliza \u03b1 = 0,8. El vector
\u20d7x se actualiza como sigue usando momentum:
\u20d7x := \u20d7x \u2212 \u03bb\u2207F(\u20d7x)
saved''',
'''Considérese que se quiere aplicar la técnica a las funciones $f_1, \\ldots, f_i, \\ldots, f_n$ en conjunto, ya que cada una de ellas tiene un significado en el contexto del problema a resolver. Estas funciones dependen del mismo vector de variables $\\vec{x} = \\{x_1, \\ldots, x_k, \\ldots, x_m\\}$. Se construye una función total

$$F(\\vec{x}) = \\sum_{i=1}^n f_i(\\vec{x})$$

a la que finalmente se aplica el descenso de gradiente. Cada subfunción (o término) $f_i$ se encapsula en un FunctionTerm que permite calcular su valor $f_i(\\hat{\\vec{x}})$, y su derivada parcial $\\dfrac{\\partial f_i(\\hat{\\vec{x}})}{\\partial \\hat{x}_k}$. Cada término tiene tambíen asignado un peso $w_i$, que determina su importancia para el cálculo del valor final.

La clase GradientDescent utiliza la funcíon de derivada parcial en cada término para construir el vector gradiente:

$$\\nabla F(\\vec{x}) = \\sum_{i=1}^n w_i \\begin{pmatrix} \\dfrac{\\partial f_i(\\vec{x})}{\\partial x_1} \\\\ \\vdots \\\\ \\dfrac{\\partial f_i(\\vec{x})}{\\partial x_k} \\\\ \\vdots \\\\ \\dfrac{\\partial f_i(\\vec{x})}{\\partial x_m} \\end{pmatrix}$$

Recordando lo expuesto en 2.6, el vector $\\vec{x}$ que minimiza localmente la función $F$ se actualiza como sigue:

$$\\vec{x} := \\vec{x} - \\lambda\\nabla F(\\vec{x})$$

Esta es la forma regular de actualizar el gradiente, pero existen muchas variantes. La clase GradientDescent, además del método regular, puede utilizar la técnica de descenso de gradiente con momentum, en que los cálculos de iteraciones pasadas tienen un peso en el cálculo del valor actual del gradiente. El valor almacenado del gradiente, $\\nabla F(\\vec{x})_{saved}$, se actualiza de la siguiente manera utilizando momentum:

$$\\nabla F(\\vec{x})_{saved} := \\alpha\\nabla F(\\vec{x})_{saved} + (1 - \\alpha)\\nabla F(\\vec{x})$$

$\\alpha$ es el factor que indica cúanto pesa el gradiente histórico $\\nabla F(\\vec{x})_{saved}$, en relación al gradiente calculado en la iteracíon actual $\\nabla F(\\vec{x})$. En este caso particular, se utiliza $\\alpha = 0{,}8$. El vector $\\vec{x}$ se actualiza como sigue usando momentum:

$$\\vec{x} := \\vec{x} - \\lambda\\nabla F(\\vec{x})_{saved}$$''',
text)

# ── STEP 8: Section 4.6.2 – IK function terms ────────────────────────────────

# Eq 4.1
text = r(
'''es:
f (\u03b8
\u20d7
) = ||eeP
\u20d7
os(\u03b8
\u20d7
) \u2212 eeTa
\u20d7
rget||2
(4.1)

\u20d7 \u20d7 \u20d7
Donde eePos(\u03b8) es la posición actual del end-effector, y eeTarget es la posición a la que''',
'''es:

$$f_1(\\vec{\\theta}) = ||\\overrightarrow{eePos}(\\vec{\\theta}) - \\overrightarrow{eeTarget}\\,||^2 \\tag{4.1}$$

Donde $\\overrightarrow{eePos}(\\vec{\\theta})$ es la posición actual del end-effector, y $\\overrightarrow{eeTarget}$ es la posición a la que''',
text)

# Eq 4.2
text = r(
'''Notar primero que eePos(\u03b8\u20d7) puede descomponerse de la siguiente manera:
eeP
\u20d7
os(\u03b8
\u20d7
) = M
\u02c6
AT\u03b8 kR\u03b8 kS\u03b8
kM
\u02c6
B
\u20d7
\u02c6
b (4.2)''',
'''Notar primero que $\\overrightarrow{eePos}(\\vec{\\theta})$ puede descomponerse de la siguiente manera:

$$\\overrightarrow{eePos}(\\vec{\\theta}) = \\hat{M}_A T_{\\theta_k} R_{\\theta_k} S_{\\theta_k} \\hat{M}_B \\hat{\\vec{b}} \\tag{4.2}$$''',
text)

# Eq 4.3
text = r(
'''constantes, quedando
MA
= M
\u02c6
AT\u03b8
k
y
\u20d7
b =
S\u03b8
kM
\u02c6
B
\u20d7
\u02c6
b = {b ,b ,b ,b }:
0 1 2 3
eeP
\u20d7
os(\u03b8
\u20d7
) =
MAR\u03b8
k
\u20d7
b (4.3)''',
'''constantes, quedando $M_A = \\hat{M}_A T_{\\theta_k}$ y $\\vec{b} = S_{\\theta_k} \\hat{M}_B \\hat{\\vec{b}} = \\{b_0, b_1, b_2, b_3\\}$:

$$\\overrightarrow{eePos}(\\vec{\\theta}) = M_A R_{\\theta_k} \\vec{b} \\tag{4.3}$$''',
text)

# Eq 4.4
text = r(
'''Los elementos de los factores de la ecuación 4.3 se agrupan con sumatorias:

b
MAR\u03b8
k
j
0i ij
\u2211

\u2211

b
MAR\u03b8
k
eeP
\u20d7
os(\u03b8

j=0 i=0
 j
1i ij

b
MAR\u03b8
k
j
3i ij

(4.4)''',
'''Los elementos de los factores de la ecuación 4.3 se agrupan con sumatorias:

$$\\overrightarrow{eePos}(\\vec{\\theta}) = \\begin{pmatrix} \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{0i}\\, b_{ij} \\\\ \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{1i}\\, b_{ij} \\\\ \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{2i}\\, b_{ij} \\\\ \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{3i}\\, b_{ij} \\end{pmatrix} \\tag{4.4}$$''',
text)

# Eq 4.5
text = r(
'''Substrayendo la posicíon objetivo:

b
MAR\u03b8
k \u2212
eeTarget0
j
0i ij

\u2211

\u2211

b
MAR\u03b8
k
\u2212
eeTarget1
eeP
\u20d7
os(\u03b8
\u20d7
) \u2212 eeTa
\u20d7
rget =

j
1i ij


(4.5)
b
MAR\u03b8
k
\u2212
eeTarget2

j=0 i=0
 j
2i ij


b
MAR\u03b8
k \u2212
eeTarget3
j
3i ij''',
'''Substrayendo la posicíon objetivo:

$$\\overrightarrow{eePos}(\\vec{\\theta}) - \\overrightarrow{eeTarget} = \\begin{pmatrix} \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{0i}\\, b_{ij} - eeTarget_0 \\\\ \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{1i}\\, b_{ij} - eeTarget_1 \\\\ \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{2i}\\, b_{ij} - eeTarget_2 \\\\ \\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{3i}\\, b_{ij} - eeTarget_3 \\end{pmatrix} \\tag{4.5}$$''',
text)

# Eq 4.6
text = r(
'''Finalmente la funcíon f reconstruida queda de la siguiente forma:

[ ]2
3 3 3
\u2211 \u2211\u2211
eeTarget
f (\u03b8
\u20d7
) = (b
MAR\u03b8
k \u2212
k
) (4.6)
1 j
ki ij

k=0 j=0 i=0''',
'''Finalmente la funcíon $f_1$ reconstruida queda de la siguiente forma:

$$f_1(\\vec{\\theta}) = \\sum_{k=0}^{3} \\left[\\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{ki}\\, b_{ij} - eeTarget_k\\right]^2 \\tag{4.6}$$''',
text)

# Eq 4.7
text = r(
'''Con esto, la parte variable de f queda claramente separada en
R\u03b8
k, y calcular la

ij
derivada parcial resulta más fácil:
[ ]
\u2202f (\u03b8
\u20d7
)
\u2211

\u2211

\u2211

eeTarget
\u2211

\u2211
3 \u2202R\u03b8
k

= 2 (b
MAR\u03b8
k
\u2212
k
) b
MA
ij
(4.7)
\u2202\u03b8
j
ki ij

j
ki
\u2202\u03b8
k k
k=0 j=0 i=0 j=0 i=0''',
'''Con esto, la parte variable de $f_1$ queda claramente separada en $[R_{\\theta_k}]_{ij}$, y calcular la derivada parcial resulta más fácil:

$$\\frac{\\partial f_1(\\vec{\\theta})}{\\partial \\theta_k} = 2 \\sum_{k=0}^{3}\\sum_{j=0}^{3}\\sum_{i=0}^{3} \\left(\\sum_{j=0}^{3}\\sum_{i=0}^{3} [M_A R_{\\theta_k}]_{ki}\\, b_{ij} - eeTarget_k\\right) [M_A]_{ki}\\, \\frac{\\partial [R_{\\theta_k}]_{ij}}{\\partial \\theta_k} \\tag{4.7}$$''',
text)

# Eq 4.8
text = r(
'''La función escogida es:
f (\u03b8
\u20d7
) = ||\u03b8
\u20d7
\u2212
\u03c9\u20d7||2
(4.8)

El vector constante \u03c9\u20d7 contiene, por cada \u03b8 , el ángulo \u03c9 original de la animacíon en
k k
el frame objetivo para la misma articulación j . La derivada es simple de calcular:
k
\u20d7
\u2202f (\u03b8)

= 2(\u03b8 \u2212 \u03c9 ) (4.9)
k k
\u2202\u03b8
k''',
'''La función escogida es:

$$f_2(\\vec{\\theta}) = ||\\vec{\\theta} - \\vec{\\omega}\\,||^2 \\tag{4.8}$$

El vector constante $\\vec{\\omega}$ contiene, por cada $\\theta_k$, el ángulo $\\omega_k$ original de la animacíon en el frame objetivo para la misma articulación $j_k$. La derivada es simple de calcular:

$$\\frac{\\partial f_2(\\vec{\\theta})}{\\partial \\theta_k} = 2(\\theta_k - \\omega_k) \\tag{4.9}$$''',
text)

# Eq 4.10-4.11
text = r(
'''Los ángulos
son análogas a las del segundo término:
f (\u03b8
\u20d7
) = ||\u03b8
\u20d7
\u2212
\u20d7\u03b3||2
(4.10)

En este caso, el vector constante \u20d7\u03b3, en lugar de contener los valores originales, contiene
los valores de frame anterior para cada j .
k
\u20d7
\u2202f (\u03b8)

= 2(\u03b8 \u2212 \u03b3 ) (4.11)
k k
\u2202\u03b8
k''',
'''Los ángulos son análogas a las del segundo término:

$$f_3(\\vec{\\theta}) = ||\\vec{\\theta} - \\vec{\\gamma}\\,||^2 \\tag{4.10}$$

En este caso, el vector constante $\\vec{\\gamma}$, en lugar de contener los valores originales, contiene los valores de frame anterior para cada $j_k$. La derivada es:

$$\\frac{\\partial f_3(\\vec{\\theta})}{\\partial \\theta_k} = 2(\\theta_k - \\gamma_k) \\tag{4.11}$$''',
text)

# Eq 4.12
text = r(
'''La funcíon final F a minimizar es:
\u20d7 \u20d7 \u20d7 \u20d7
F(\u03b8) = af (\u03b8) + bf (\u03b8) + cf (\u03b8) (4.12)
1 2 3''',
'''La funcíon final $F$ a minimizar es:

$$F(\\vec{\\theta}) = a\\,f_1(\\vec{\\theta}) + b\\,f_2(\\vec{\\theta}) + c\\,f_3(\\vec{\\theta}) \\tag{4.12}$$''',
text)

# Coeff a value
text = r(
'''Los valores usados son a =

, b = 2, y c = 4. El valor rigHeight corresponde a
[rigHeight][δp]''',
'''Los valores usados son $a = \\dfrac{1}{rigHeight \\cdot \\delta_p}$, $b = 2$, y $c = 4$. El valor $rigHeight$ corresponde a''',
text)

# Eq 4.13-4.14
text = r(
'''alcanzada, se calcula como \u03b4p =
2lsin(\u03b4\u03b8).
Al ser \u03b4\u03b8 pequeño, puede aproximarse la función

seno a su argumento, con lo que \u03b4p \u2248 l\u03b4\u03b8. Reordenando y considerando el cuadrado de la
distancia:
\u03b4p
l \u2248 (4.13)
\u03b4\u03b8
\u03b4p2
l\u03b4p \u2248 (4.14)
\u03b4\u03b8''',
'''alcanzada, se calcula como $\\delta p = 2l\\sin(\\delta\\theta)$. Al ser $\\delta\\theta$ pequeño, puede aproximarse la función seno a su argumento, con lo que $\\delta p \\approx l\\delta\\theta$. Reordenando y considerando el cuadrado de la distancia:

$$l \\approx \\frac{\\delta p}{\\delta\\theta} \\tag{4.13}$$

$$l\\,\\delta p \\approx \\frac{\\delta p^2}{\\delta\\theta} \\tag{4.14}$$''',
text)

# Eq init theta = gamma
text = r(
'''se escoge entonces inicializar el vector de variables como
sigue:
\u20d7
\u03b8 = \u20d7\u03b3''',
'''se escoge entonces inicializar el vector de variables como sigue:

$$\\vec{\\theta} = \\vec{\\gamma}$$''',
text)

# ── STEP 9: Section 4.8.1 – LIC velocity equations ───────────────────────────
text = r(
'''2. getPointVelocity: Calcular la velocidad de un punto p\u20d7 por la izquierda y por la derecha,
k
donde las ecuaciones son
p\u20d7
k
\u2212p\u20d7
k\u22121
y
p\u20d7
k+1
\u2212p\u20d7
k
respectivamente.
t \u2212t t \u2212t
k k\u22121 k+1 k''',
r'''2. getPointVelocity: Calcular la velocidad de un punto $\vec{p}_k$ por la izquierda y por la derecha, donde las ecuaciones son $\dfrac{\vec{p}_k - \vec{p}_{k-1}}{t_k - t_{k-1}}$ y $\dfrac{\vec{p}_{k+1} - \vec{p}_k}{t_{k+1} - t_k}$ respectivamente.''',
text)

# ── STEP 10: Section 4.8.5 – Trajectory correction (eq 4.15-4.16) ────────────
text = r(
'''La funcíon a
minimizar es entonces:
n
\u2211[ ]
f(P
\u20d7
) = ||lV el(p
\u20d7
T ) \u2212 lV el(p
\u20d7
B
)||2
+ ||rV el(p
\u20d7
T ) \u2212 rV el(p
\u20d7
B
)||2
(4.15)
T
i i i i
i=1''',
r'''La funcíon a minimizar es entonces:

$$f(\vec{P}_T) = \sum_{i=1}^{n} \left[ ||lVel(\vec{p}_{T_i}) - lVel(\vec{p}_{B_i})||^2 + ||rVel(\vec{p}_{T_i}) - rVel(\vec{p}_{B_i})||^2 \right] \tag{4.15}$$''',
text)

text = r(
'''[ ]
\u20d7 \u20d7 \u20d7 \u20d7 \u20d7
\u2202f(P ) lV el(pT ) \u2212 lV el(pB ) rV el(pT ) \u2212 rV el(pB )
T
= 2

w

w
+

w

w
(4.16)''',
r'''$$\frac{\partial f(\vec{P}_T)}{\partial w} = 2\left[\frac{\partial\, lVel(\vec{p}_T)}{\partial w} - \frac{\partial\, lVel(\vec{p}_B)}{\partial w} + \frac{\partial\, rVel(\vec{p}_T)}{\partial w} - \frac{\partial\, rVel(\vec{p}_B)}{\partial w}\right] \tag{4.16}$$''',
text)

# ── STEP 11: Annex B.1 – Rotation matrix ─────────────────────────────────────
text = r(
'''De [24] se extrae la ecuación para una matriz de 3x3, donde \u20d7a es el eje de rotación y \u03b8 el
ángulo:

cos\u03b8 +
a2(1
\u2212 cos\u03b8) a a (1 \u2212 cos\u03b8) \u2212 a sin\u03b8 a a (1 \u2212 cos\u03b8) + a sin\u03b8

0 1 2 0 2 1
R(\u03b8,\u20d7a) = a

a

(1 \u2212 cos\u03b8) + a

sin\u03b8 cos\u03b8 +
a2

(1 \u2212 cos\u03b8) a

a

(1 \u2212 cos\u03b8) \u2212 a

sin\u03b8
a a (1 \u2212 cos\u03b8) \u2212 a sin\u03b8 a a (1 \u2212 cos\u03b8) + a sin\u03b8 cos\u03b8 +
a2(1
\u2212 cos\u03b8)
0 2 1 1 2 0

Expandiendo a 4x4 y derivando con respecto a \u03b8:

\u2212sin\u03b8 +
a2sin\u03b8
a a sin\u03b8 \u2212 a cos\u03b8 a a sin\u03b8 + a cos\u03b8 0

0 1 2 0 2 1
\u2202R(\u03b8,\u20d7a)
=


a

a

sin\u03b8 + a

cos\u03b8 \u2212sin\u03b8 +
a2

sin\u03b8 a

a

sin\u03b8 \u2212 a

cos\u03b8 0

\u2202\u03b8
a

a

sin\u03b8 \u2212 a

cos\u03b8 a

a

sin\u03b8 + a

cos\u03b8 \u2212sin\u03b8 +
a2

sin\u03b8 0
0 0 0 0''',
r'''De [24] se extrae la ecuación para una matriz de 3x3, donde $\vec{a} = (a_0, a_1, a_2)$ es el eje de rotación y $\theta$ el ángulo:

$$R(\theta,\vec{a}) = \begin{pmatrix} \cos\theta + a_0^2(1-\cos\theta) & a_0 a_1(1-\cos\theta) - a_2\sin\theta & a_0 a_2(1-\cos\theta) + a_1\sin\theta \\ a_1 a_0(1-\cos\theta) + a_2\sin\theta & \cos\theta + a_1^2(1-\cos\theta) & a_1 a_2(1-\cos\theta) - a_0\sin\theta \\ a_2 a_0(1-\cos\theta) - a_1\sin\theta & a_2 a_1(1-\cos\theta) + a_0\sin\theta & \cos\theta + a_2^2(1-\cos\theta) \end{pmatrix}$$

Expandiendo a 4x4 y derivando con respecto a $\theta$:

$$\frac{\partial R(\theta,\vec{a})}{\partial\theta} = \begin{pmatrix} -\sin\theta + a_0^2\sin\theta & a_0 a_1\sin\theta - a_2\cos\theta & a_0 a_2\sin\theta + a_1\cos\theta & 0 \\ a_1 a_0\sin\theta + a_2\cos\theta & -\sin\theta + a_1^2\sin\theta & a_1 a_2\sin\theta - a_0\cos\theta & 0 \\ a_2 a_0\sin\theta - a_1\cos\theta & a_2 a_1\sin\theta + a_0\cos\theta & -\sin\theta + a_2^2\sin\theta & 0 \\ 0 & 0 & 0 & 0 \end{pmatrix}$$''',
text)

# ── STEP 12: Minor inline cleanup ─────────────────────────────────────────────

# Remove isolated combining-arrow lines (floating ⃗ on own line between paragraphs)
import re
# Only remove ⃗ that sits alone on a line (surrounded by blank/newline context)
text = re.sub(r'\n\u20d7\n', '\n', text)
text = re.sub(r'^\u20d7\n', '', text, flags=re.MULTILINE)

# Fix trailing subscript number lines that are clearly orphaned (e.g. "1 2\n" "i i\n")
# These are hard to do globally, but we can clean up common remnants left outside replaced blocks

with open(MD_PATH, 'w', encoding='utf-8') as f:
    f.write(text)

print("Done — equations converted to LaTeX.")
