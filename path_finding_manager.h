//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H

#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>
#include <limits>
#include <cmath>
#include <chrono>

// Usamos sólo los tipos de std que necesitamos para mantener el código compacto.
using std::unordered_map;
using std::set;
using std::vector;
using std::size_t;
using std::numeric_limits;

// Algoritmo de búsqueda seleccionado por el usuario
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
    vector<sfLine> path;
    vector<sfLine> visited_edges;
    int render_counter = 0;  // Contador para renderizado optimizado
    
    // Estadísticas de ejecución
    struct ExecutionStats {
        double path_distance = 0.0;
        size_t nodes_visited = 0;
        size_t edges_explored = 0;
        double execution_time_ms = 0.0;
        bool path_found = false;
    } stats;
    
    Node* last_dest = nullptr;  // Para cache de heurística

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            const double eps = 1e-12;
            if (std::fabs(dist - other.dist) > eps) {
                return dist < other.dist;
            }
            if (node != nullptr && other.node != nullptr) {
                return node->id < other.node->id;
            }
            return node < other.node;
        }
    };

    // Vecino alcanzable respetando la dirección de la arista
    Node* get_valid_neighbor(Edge* edge, Node* current) {
        if (edge->src == current) {
            return edge->dest;
        } else if (!edge->one_way && edge->dest == current) {
            return edge->src;
        }
        return nullptr;
    }

    // Distancia euclidiana entre dos nodos
    static double heuristic(Node* a, Node* b) {
        const double dx = a->coord.x - b->coord.x;
        const double dy = a->coord.y - b->coord.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Elimina todas las entradas de un nodo en el set (para actualizar su prioridad)
    static void remove_old_entries(set<Entry>& pq, Node* node) {
        for (auto it = pq.begin(); it != pq.end(); ) {
            if (it->node == node) {
                it = pq.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Cache de heurística para el destino actual
    unordered_map<Node*, double> heuristic_cache;

    void compute_heuristic_cache(Graph &graph) {
        // Solo recalcular si el destino cambió
        if (dest == nullptr) {
            heuristic_cache.clear();
            last_dest = nullptr;
            return;
        }
        
        if (dest == last_dest && !heuristic_cache.empty()) {
            return;  // Cache válido, no recalcular
        }
        
        heuristic_cache.clear();
        last_dest = dest;
        for (auto &p : graph.nodes) {
            Node* node = p.second;
            heuristic_cache[node] = heuristic(node, dest);
        }
    }

    void dijkstra(Graph &graph) {
        auto start_time = std::chrono::high_resolution_clock::now();
        stats.nodes_visited = 0;
        stats.edges_explored = 0;
        
        unordered_map<Node *, double> distances;
        unordered_map<Node *, bool> visited;
        unordered_map<Node *, Node *> parent;
        set<Entry> pq;

        for (auto &[id, node] : graph.nodes) {
            distances[node] = numeric_limits<double>::max();
            visited[node] = false;
        }

        distances[src] = 0;
        pq.insert({src, 0});

        while (!pq.empty()) {
            Entry current_entry = *pq.begin();
            pq.erase(pq.begin());

            Node *current = current_entry.node;

            if (visited[current] || current_entry.dist > distances[current]) {
                continue;
            }
            
            visited[current] = true;
            stats.nodes_visited++;
            current->color = sf::Color::Yellow;
            
            render_counter++;
            if (render_counter % 50 == 0) {
                render();
            }

            if (current == dest) break;

            for (Edge *edge : current->edges) {
                Node *neighbor = get_valid_neighbor(edge, current);
                if (neighbor == nullptr) continue;

                if (!visited[neighbor]) {
                    double new_dist = distances[current] + edge->length;

                    if (new_dist < distances[neighbor]) {
                        remove_old_entries(pq, neighbor);
                        distances[neighbor] = new_dist;
                        parent[neighbor] = current;
                        pq.insert({neighbor, new_dist});
                        stats.edges_explored++;
                        visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color::Magenta, 1.0f);
                    }
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        stats.execution_time_ms =
                std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        render();
        set_final_path(parent);
    }

    void best_first_search(Graph &graph) {
        auto start_time = std::chrono::high_resolution_clock::now();
        stats.nodes_visited = 0;
        stats.edges_explored = 0;
        
        unordered_map<Node *, bool> visited;
        unordered_map<Node *, Node *> parent;
        set<Entry> pq;

        compute_heuristic_cache(graph);

        for (auto &pr : graph.nodes) {
            visited[pr.second] = false;
        }

        double h_src = heuristic_cache.count(src) ? heuristic_cache[src] : heuristic(src, dest);
        pq.insert({src, h_src});

        while (!pq.empty()) {
            Entry current_entry = *pq.begin();
            pq.erase(pq.begin());

            Node *current = current_entry.node;

            if (visited[current]) continue;
            visited[current] = true;
            stats.nodes_visited++;
            current->color = sf::Color::Yellow;
            
            render_counter++;
            if (render_counter % 50 == 0) {
                render();
            }

            if (current == dest) break;

            for (Edge *edge : current->edges) {
                Node *neighbor = get_valid_neighbor(edge, current);
                if (neighbor == nullptr) continue;

                if (!visited[neighbor]) {
                    remove_old_entries(pq, neighbor);
                    parent[neighbor] = current;
                    double h_neighbor = heuristic_cache.count(neighbor) ? heuristic_cache[neighbor] : heuristic(neighbor, dest);
                    pq.insert({neighbor, h_neighbor});
                    stats.edges_explored++;
                    visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color::Cyan, 1.0f);
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        stats.execution_time_ms =
                std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        render();
        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        auto start_time = std::chrono::high_resolution_clock::now();
        stats.nodes_visited = 0;
        stats.edges_explored = 0;
        
        unordered_map<Node *, double> g_score;
        unordered_map<Node *, bool> closed_set;
        unordered_map<Node *, Node *> parent;
        set<Entry> open_set;

        for (auto &[id, node] : graph.nodes) {
            g_score[node] = numeric_limits<double>::max();
            closed_set[node] = false;
        }

        compute_heuristic_cache(graph);

        g_score[src] = 0;
        double h_src = heuristic_cache.count(src) ? heuristic_cache[src] : heuristic(src, dest);
        double f_src = h_src;
        open_set.insert({src, f_src});

        while (!open_set.empty()) {
            Entry current_entry = *open_set.begin();
            open_set.erase(open_set.begin());

            Node *current = current_entry.node;

            if (closed_set[current]) continue;
            
            double h_current = heuristic_cache.count(current) ? heuristic_cache[current] : heuristic(current, dest);
            double expected_f = g_score[current] + h_current;
            if (fabs(current_entry.dist - expected_f) > 1e-9) {
                continue;
            }
            
            closed_set[current] = true;
            stats.nodes_visited++;
            current->color = sf::Color::Yellow;
            
            render_counter++;
            if (render_counter % 50 == 0) {
                render();
            }

            if (current == dest) break;

            for (Edge *edge : current->edges) {
                Node *neighbor = get_valid_neighbor(edge, current);
                if (neighbor == nullptr) continue;

                if (!closed_set[neighbor]) {
                    double tentative_g = g_score[current] + edge->length;

                    if (tentative_g < g_score[neighbor]) {
                        parent[neighbor] = current;
                        g_score[neighbor] = tentative_g;
                        double h_neighbor = heuristic_cache.count(neighbor) ? heuristic_cache[neighbor] : heuristic(neighbor, dest);
                        double f_neighbor = tentative_g + h_neighbor;
                        open_set.insert({neighbor, f_neighbor});
                        stats.edges_explored++;
                        visited_edges.emplace_back(current->coord, neighbor->coord, sf::Color::Green, 1.0f);
                    }
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        stats.execution_time_ms =
                std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        render();
        set_final_path(parent);
    }

    // Renderiza el estado actual del algoritmo (ligeramente acelerado)
    void render() {
        sf::sleep(sf::milliseconds(5));

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

    // Reconstruye el camino final desde dest hasta src usando el mapa de padres
    void set_final_path(unordered_map<Node *, Node *> &parent) {
        stats.path_found = false;
        stats.path_distance = 0.0;
        
        if (dest == nullptr) return;
        if (parent.find(dest) == parent.end()) return;

        vector<sfLine> tmp;
        Node* current = dest;
        while (current != nullptr && parent.find(current) != parent.end()) {
            Node* p = parent[current];
            tmp.emplace_back(p->coord, current->coord, sf::Color::Red, 2.0f);
            double dx = p->coord.x - current->coord.x;
            double dy = p->coord.y - current->coord.y;
            stats.path_distance += sqrt(dx * dx + dy * dy);
            current = p;
        }

        reverse(tmp.begin(), tmp.end());
        path = move(tmp);
        stats.path_found = true;
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}
    
    const ExecutionStats& get_stats() const { return stats; }

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }
        // Asociar grafo para renderizado intra-algoritmo
        graph_ptr = &graph;

        // Limpieza de estado visual previo
        path.clear();
        visited_edges.clear();
        render_counter = 0;  // Resetear contador de renderizado

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
