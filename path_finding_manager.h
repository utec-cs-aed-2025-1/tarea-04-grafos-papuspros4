//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>
#include <queue>
#include <limits>
#include <cmath>

// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    BestFirstSearch,
    AStar
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    Graph *graph_ptr = nullptr;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
    };

    // Heurística: distancia euclidiana entre dos nodos
    double heuristic(Node* a, Node* b) const {
        double dx = a->coord.x - b->coord.x;
        double dy = a->coord.y - b->coord.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, double> distances;
        std::unordered_map<Node *, bool> visited;
        std::unordered_map<Node *, Node *> parent;
        std::set<Entry> pq;

        // Inicializar distancias a infinito
        for (auto &[id, node] : graph.nodes) {
            distances[node] = std::numeric_limits<double>::max();
            visited[node] = false;
        }

        distances[src] = 0;
        pq.insert({src, 0});

        while (!pq.empty()) {
            Entry current_entry = *pq.begin();
            pq.erase(pq.begin());

            Node *current = current_entry.node;

            if (visited[current]) continue;
            visited[current] = true;
            current->color = sf::Color::Yellow;
            render();

            if (current == dest) break;

            // Explorar vecinos
            for (Edge *edge : current->edges) {
                Node *neighbor = edge->dest;
                if (edge->src != current) {
                    neighbor = edge->src;
                }

                if (!visited[neighbor]) {
                    double new_dist = distances[current] + edge->length;

                    if (new_dist < distances[neighbor]) {
                        distances[neighbor] = new_dist;
                        parent[neighbor] = current;
                        pq.insert({neighbor, new_dist});

                        // Visualizar la arista explorada
                        visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color::Magenta, 1.0f);
                    }
                }
            }
        }

        set_final_path(parent);
    }

    void best_first_search(Graph &graph) {
        std::unordered_map<Node *, bool> visited;
        std::unordered_map<Node *, Node *> parent;
        std::set<Entry> pq;

        // Inicializar visitados
        for (auto &[id, node] : graph.nodes) {
            visited[node] = false;
        }

        // Usar solo la heurística h(n) para ordenar
        double h_src = heuristic(src, dest);
        pq.insert({src, h_src});

        while (!pq.empty()) {
            Entry current_entry = *pq.begin();
            pq.erase(pq.begin());

            Node *current = current_entry.node;

            if (visited[current]) continue;
            visited[current] = true;
            current->color = sf::Color::Yellow;
            render();

            if (current == dest) break;

            // Explorar vecinos
            for (Edge *edge : current->edges) {
                Node *neighbor = edge->dest;
                if (edge->src != current) {
                    neighbor = edge->src;
                }

                if (!visited[neighbor]) {
                    parent[neighbor] = current;
                    double h_neighbor = heuristic(neighbor, dest);
                    pq.insert({neighbor, h_neighbor});

                    // Visualizar la arista explorada
                    visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color::Cyan, 1.0f);
                }
            }
        }

        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, double> g_score;      // Costo real desde src
        std::unordered_map<Node *, bool> closed_set;     // Visitados
        std::unordered_map<Node *, Node *> parent;
        std::set<Entry> open_set;

        // Inicializar g_score
        for (auto &[id, node] : graph.nodes) {
            g_score[node] = std::numeric_limits<double>::max();
            closed_set[node] = false;
        }

        g_score[src] = 0;
        double h_src = heuristic(src, dest);
        double f_src = h_src;
        open_set.insert({src, f_src});

        while (!open_set.empty()) {
            Entry current_entry = *open_set.begin();
            open_set.erase(open_set.begin());

            Node *current = current_entry.node;

            if (closed_set[current]) continue;
            closed_set[current] = true;
            current->color = sf::Color::Yellow;
            render();

            if (current == dest) break;

            // Explorar vecinos
            for (Edge *edge : current->edges) {
                Node *neighbor = edge->dest;
                if (edge->src != current) {
                    neighbor = edge->src;
                }

                if (!closed_set[neighbor]) {
                    double tentative_g = g_score[current] + edge->length;

                    if (tentative_g < g_score[neighbor]) {
                        parent[neighbor] = current;
                        g_score[neighbor] = tentative_g;
                        double h_neighbor = heuristic(neighbor, dest);
                        double f_neighbor = tentative_g + h_neighbor;
                        open_set.insert({neighbor, f_neighbor});

                        // Visualizar la arista explorada
                        visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color::Green, 1.0f);
                    }
                }
            }
        }

        set_final_path(parent);
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(10));

        // Si hay un graph asociado, dibujar su estado y las aristas visitadas
        window_manager->clear();
        if (graph_ptr != nullptr) {
            graph_ptr->draw();
        }

        // Dibujar las aristas visitadas hasta ahora
        for (sfLine &line : visited_edges) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial (verde)
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final (celeste)
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }

        // Mostrar el frame actual
        window_manager->display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        Node* current = dest;

        while (current != nullptr && parent.find(current) != parent.end()) {
            Node* p = parent[current];
            path.emplace_back(current->coord, p->coord, sf::Color::Red, 2.0f);
            current = p;
        }
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }
        // Asociar grafo para renderizado intra-algoritmo
        graph_ptr = &graph;

        // Limpieza de estado visual previo
        path.clear();
        visited_edges.clear();

        // Resetear colores/radio de todos los nodos
        for (auto & [id, node] : graph.nodes) {
            node->reset();
        }
        // Recolorear src y dest para que sean visibles
        if (src) {
            src->color = sf::Color::Green;
            src->radius = 3.0f;
        }
        if (dest) {
            dest->color = sf::Color::Cyan;
            dest->radius = 3.0f;
        }

        // Ejecutar algoritmo seleccionado
        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case BestFirstSearch:
                best_first_search(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            default:
                break;
        }

        // Desasociar grafo
        graph_ptr = nullptr;
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
