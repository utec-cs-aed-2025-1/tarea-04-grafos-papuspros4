[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/5zgGDtf4)
[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=21627334&assignment_repo_type=AssignmentRepo)
# Tarea de Grafos

## Integrantes: 
- 1 Adriana Lucia Castro Quiñones
- 2 Benjamin Mario Augusto Suarez Arzapalo


## Objetivo: 
El objetivo de esta tarea es implementar un **Path Finder** para la ciudad de Lima. 

<p align="center">
    <img src=https://github.com/utec-cs-aed/homework_graph/assets/79115974/b63f69db-17eb-417a-8aa1-8483d8dcdaf0 / >
</p>

## Dependencias

Para esta tarea se solicita utilizar ```C++17``` y la librería ```SFML 2.5```

- Para instalar ```SFML 2.5```:

    - [Windows](https://www.youtube.com/watch?v=HkPRG0vfObc)
    - [MacOS y Linux](https://www.youtube.com/playlist?list=PLvv0ScY6vfd95GMoMe2zc4ZgGxWYj3vua)

Cuando se instale la librería, probar que las siguientes líneas del ```CMakeLists.txt``` encuentren la librería adecuadamente.
```cmake
find_package(SFML 2.5 COMPONENTS graphics window REQUIRED)
if(SFML_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window)
else()
    message("SFML not found")
endif()
```

## Dataset
El dataset consiste de dos csv:

- *nodes.csv*

    ![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/6a68cf06-196a-4605-83a7-3183e9a3f0ec)


- *edges.csv*

    ![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/247bbbd7-6203-45f4-8196-fcb0434b0f1d)


## Algoritmos
Se les solicita implementar tres algoritmos para busqueda en grafos

- *Dijkstra*

- *Best First Search*

- *A**

Además:
- Analice la complejidad computacional de los tres algoritmos de acuerdo a su propia implementación.
- Puede considere como heuristica la distancia en linea recta.
- **Debe realizar un pequeño video (2 min) mostrando la funcionalidad visual de cada algoritmo**

## Diagrama de clases UML 

![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/f5a3d89e-cb48-4715-b172-a17e6e27ee24)

----------
> **Créditos:** Juan Diego Castro Padilla [juan.castro.p@utec.edu.pe](mailto:juan.castro.p@utec.edu.pe)


-----------

# Análisis de Complejidad

Este proyecto implementa tres algoritmos de búsqueda de caminos: Dijkstra, Greedy Best‑First (GBFS) y A*.
El análisis siguiente está adaptado a la implementación presente en `path_finding_manager.h` (uso de
`std::set` como cola priorizada, caché de heurística, y visualización intra‑algoritmo).

## Resumen de estructuras relevantes
- Grafo: `graph.nodes` (map de id → `Node*`), cada `Node` contiene su lista de `Edge*`.
- Cola de prioridad: `std::set<Entry>` donde `Entry` contiene `Node*` y `double` (valor de ordenación);
  el operador `<` incluye un tie‑break por `node->id` para evitar pérdidas por empates.
- Heurística: distancia euclidiana; la implementación mantiene `heuristic_cache` precomputada por destino.
- Visualización: `render()` dibuja el grafo y las aristas visitadas durante la ejecución (incluye un `sf::sleep`).

## Complejidad por algoritmo (peor caso)

### Dijkstra
- Temporal: O((V + E) log V)
  - Inicializa distancias en O(V).
  - Cada extracción/inserción en la cola priorizada es O(log V).
  - Cada arista se considera al menos una vez en la relajación (con costo amortizado por log V).
- Espacial: O(V) (mapas de distancias/parent/visited, y la cola). Si se contabiliza la visualización, `visited_edges` puede crecer hasta O(E).

### Greedy Best‑First (GBFS)
- Temporal (peor caso): O((V + E) log V)
  - Precomputa heurística en O(V) (una sola vez por ejecución).
  - Usa la heurística h(n) para ordenar la cola; en el peor caso expande todos los nodos y aristas.
- Espacial: O(V) (+O(E) si se cuentan las aristas visitadas visualizadas).

### A*
- Temporal (peor caso): O((V + E) log V)
  - Mantiene `g_score` y utiliza f(n)=g(n)+h(n) en la cola; estructura de operaciones similar a Dijkstra.
  - Con heurística admisible (euclidiana) suele explorar mucho menos nodos en la práctica.
- Espacial: O(V) (g_score, parent, open/closed sets) y O(V) adicional para `heuristic_cache`.
---

## Análisis detallado por algoritmo (implementación propia)

En esta sección se desglosa la complejidad de cada algoritmo según esta implementación concreta  
(`std::set<Entry>` como cola de prioridad, heurística euclidiana cacheada y renderizado periódico del grafo).

Sea:

- \( V = |\text{nodos}| \)  
- \( E = |\text{aristas}| \)  
- `timmer`: número de iteraciones entre llamadas a `render()`  
- \( L \): número de aristas efectivamente relajadas/visitadas (en el peor caso \( L \approx E \))

---

## 1. Dijkstra

### **Inicialización**
- Recorrer `graph.nodes` para asignar distancias y visitados: \( O(V) \)  
- Insertar `src` en el `set<Entry>`: \( O(\log V) \)

**Total:**

\[
T_{\text{init}} = O(V)
\]

### **Bucle principal**
- Extraer el mejor nodo del set: como máximo \( V \) veces, costo \( O(V \log V) \)
- Examinar cada arista una vez; relajación exitosa → remove + insert: \( O(E \log V) \)

### **Complejidad total (sin render)**

\[
T_{\text{Dijkstra}} = O((V+E)\log V),\qquad
S_{\text{Dijkstra}} = O(V)
\]

### **Costo del renderizado**

Cada `render()` cuesta:

\[
T_{\text{render}} = O(V+E)
\]

Si se llama \( R \) veces:

\[
R \approx \left\lfloor \frac{L}{\text{timmer}} \right\rfloor + 1
\]

Costo total:

\[
T_{\text{render,total}} = O\left(\frac{L}{\text{timmer}}(V+E)\right)
\]

Peor caso \( L \approx E \):

\[
T_{\text{render,total}} = O\left(\frac{E}{\text{timmer}}(V+E)\right)
\]

---

## 2. A*

A* mantiene:
- `g_score[node]`
- heurística \( h(n) \) cacheada
- \( f(n)=g(n)+h(n) \) en `Entry::dist`

### **Costes básicos**
- Inicialización: \( O(V) \)  
- Extraer mejor nodo: \( O(V \log V) \)  
- Relajaciones: \( O(E \log V) \)  
- Precálculo de heurística: \( O(V) \)

### **Complejidad sin render**

\[
T_{\text{A*}} = O((V+E)\log V),\qquad
S_{\text{A*}} = O(V)
\]

### **Complejidad práctica**

\[
T_{\text{A* práctico}} \approx O((V' + E')\log V)
\]

### **Renderizado**

\[
T_{\text{render,A*}} = O\left(\frac{E'}{\text{timmer}}(V+E)\right)
\]

---

## 3. Greedy Best-First Search

Best-First usa solo \( h(n) \) para priorizar la expansión.

### **Costes básicos**
- Inicialización: \( O(V) \)  
- Extracción del mejor nodo: \( O(V \log V) \)  
- Relajación por arista: \( O(E \log V) \)

### **Complejidad sin render**

\[
T_{\text{BestFirst}} = O((V+E)\log V),\qquad
S_{\text{BestFirst}} = O(V)
\]

### **Renderizado**

\[
T_{\text{render,BF}} = O\left(\frac{L}{\text{timmer}}(V+E)\right)
\]

---

## 4. Comparación general

Peor caso teórico:

\[
T_{\text{peor}} = O((V+E)\log V)
\]

Diferencias prácticas:

- **Dijkstra** puede explorar casi todo el grafo.  
- **A\*** explora menos con buena heurística.  
- **Greedy Best-First** es rápido pero no óptimo.  
- Si `timmer` es pequeño, el renderizado puede dominar.
