package org.firstinspires.ftc.teamcode.sp;

import org.firstinspires.ftc.teamcode.graphs.Edge;
import org.firstinspires.ftc.teamcode.graphs.EdgeWeightedGraph;

import java.util.HashMap;
import java.util.Map;

// 5080 edges

public class Grid {
    private EdgeWeightedGraph G;
    private HashMap<String, Integer> dict;

    private double vertWeight = 1;
    private double strafeWeight = 1.2345679;
    // private double diagonalWeight = 2.59;
    private double diagonalWeight = 1.589;

    public Grid() {
        G = new EdgeWeightedGraph(25 * 25);
        dict = new HashMap<>();

        for (int i = 0; i < 25; i++) {
            for (int j = 0; j < 25; j++) {

                char letter = "ABCDEFGHIJKLMNOPQRSTUVWXYZ".charAt(i);
                int index = i * 25 + j;

                dict.put(letter + "" + (j + 1), index);

                if (i != 24 && j != 24 && !checkSubmersible(i, j)) {
                    G.addEdge(new Edge(index, index + 1, strafeWeight));
                    G.addEdge(new Edge(index, index + 25, vertWeight));
                    G.addEdge(new Edge(index, index + 26, diagonalWeight));
                }

                else if (i == 24 && j != 24 && !checkSubmersible(i, j)) {
                    G.addEdge(new Edge(index, index + 1, strafeWeight));
                }

                else if (i != 24 && j == 24 && !checkSubmersible(i, j)) {
                    G.addEdge(new Edge(index, index + 25, vertWeight));
                }

                else if (checkSubmersible(i, j)) {
                    if (i != 16 && j != 16) {
                        G.addEdge(new Edge(index, index + 1, Double.POSITIVE_INFINITY));
                        G.addEdge(new Edge(index, index + 25, Double.POSITIVE_INFINITY));
                        G.addEdge(new Edge(index, index + 26, Double.POSITIVE_INFINITY));
                    }

                    else if (i == 16 && j != 16) {
                        G.addEdge(new Edge(index, index + 1, Double.POSITIVE_INFINITY));
                    }

                    else if (i != 16) {
                        G.addEdge(new Edge(index, index + 25, Double.POSITIVE_INFINITY));
                    }
                }
            }
        }
    }

    public HashMap<String, Integer> getMap() {
        return dict;
    }

    public EdgeWeightedGraph getGraph() {
        return G;
    }

    private boolean checkSubmersible(int i, int j) {
        return (i >= 8 && j >= 8 && i <= 16 && j <= 16);
    }
}