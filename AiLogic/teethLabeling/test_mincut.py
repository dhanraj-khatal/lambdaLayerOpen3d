import networkx as nx
import numpy as np
from matplotlib import pyplot as plt


if __name__ == "__main__":
    G = nx.DiGraph()
    G.add_edge("x", "a", capacity=2.0)
    G.add_edge("x", "b", capacity=2.0)
    G.add_edge("a", "c", capacity=2.0)
    G.add_edge("b", "c", capacity=2.0)
    G.add_edge("b", "d", capacity=2.0)
    G.add_edge("d", "e", capacity=1.0)
    G.add_edge("c", "y", capacity=1.0)
    G.add_edge("e", "y", capacity=1.0)

    cut_value, partition = nx.minimum_cut(G, _s="x", _t="y")
    reachable, non_reachable = partition

    print(f'{reachable=}')
    print(f'{non_reachable=}')
    print("cut vslue", cut_value)

    # 3d spring layout
    pos = nx.spring_layout(G, dim=3, seed=779)
    # Extract node and edge positions from the layout
    node_xyz = np.array([pos[v] for v in sorted(G)])
    edge_xyz = np.array([(pos[u], pos[v]) for u, v in G.edges()])

    # Create the 3D figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plot the nodes - alpha is scaled by "depth" automatically
    ax.scatter(*node_xyz.T, s=100, ec="w")

    # Plot the edges
    for vizedge in edge_xyz:
        ax.plot(*vizedge.T, color="tab:gray")


    def _format_axes(ax):
        """Visualization options for the 3D axes."""
        # Turn gridlines off
        ax.grid(False)
        # Suppress tick labels
        for dim in (ax.xaxis, ax.yaxis, ax.zaxis):
            dim.set_ticks([])
        # Set axes labels
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

    _format_axes(ax)
    fig.tight_layout()
    plt.show()