package org.opentrackingtools.graph.impl;

import org.geotools.graph.path.Path;
import org.geotools.graph.structure.Graph;
import org.geotools.graph.structure.Graphable;
import org.geotools.graph.structure.Node;
import org.geotools.graph.traverse.GraphTraversal;
import org.geotools.graph.traverse.GraphWalker;
import org.geotools.graph.traverse.basic.BasicGraphTraversal;
import org.geotools.graph.traverse.standard.AStarIterator;
import org.geotools.graph.traverse.standard.AStarIterator.AStarFunctions;

/**
 * 
 * Copied from geotools AStarShortestPathFinder.java
 * 
 * @author bwillard
 * 
 */
public class CustomAStarShortestPathFinder implements GraphWalker {
  /** Graphs to calculate paths for **/
  private final Graph m_graph;

  /** Graph traversal used for the A Star iteration **/
  private final GraphTraversal m_traversal;

  /** Underling A Star iterator **/
  private final AStarIterator m_iterator;

  /**  */
  private final Node m_target;

  /**
   * Constructs a new path finder
   * 
   * @param graph Graph where we will perform the search.
   * @param source Node to calculate path from.
   * @param target Node to calculate path to.
   * @param weighter Associates weights with edges in the graph.
   */
  public CustomAStarShortestPathFinder(Graph graph, Node source, Node target,
      AStarFunctions afuncs) {
    m_graph = graph;
    m_target = target;
    m_iterator = new AStarIterator(source, afuncs);
    m_traversal = new BasicGraphTraversal(graph, this, m_iterator);
  }

  /**
   * Performs the graph traversal and calculates the shortest path from the
   * source node to destiny node in the graph.
   */
  public void calculate() {
    m_traversal.init();
    m_traversal.traverse();
  }

  /**
   * 
   * @see GraphWalker#visit(Graphable, GraphTraversal)
   */
  @Override
  public int visit(Graphable element, GraphTraversal traversal) {
    if (element.equals(m_target)) {
      return (GraphTraversal.STOP);
    } else {
      return (GraphTraversal.CONTINUE);
    }
  }

  /**
   * Returns a path <B>from</B> the target <B>to</B> the source. If the desired
   * path is the opposite (from the source to the target), the <i>reverse</i> or
   * the <i>riterator</i> methods from the <b>Path<b> class can be used.
   * 
   * @see Path#riterator()
   * @see Path#reverse()
   * 
   * @return A path from the target to the source.
   * @throws WrongPathException
   */
  public Path getPath() {
    final Path path = new Path();

    path.add(m_target);
    Node parent = m_iterator.getParent(m_target);
    while (parent != null) {
      path.add(parent);
      parent = m_iterator.getParent(parent);
    }
    if (!path.getLast().equals(m_iterator.getSource())) {
      return null;
    }
    return (path);
  }

  /**
   * Does nothing.
   * 
   * @see GraphWalker#finish()
   */
  @Override
  public void finish() {
  }

}