package org.opentrackingtools.graph;

import java.util.Iterator;

import org.geotools.graph.structure.basic.BasicDirectedNode;

/**
 * Simply changes getRelated to return out nodes, so that generic traversal code
 * in geotools will obey direction.
 * 
 * @author bwillard
 * 
 */
public class StrictDirectedNode extends BasicDirectedNode {

  /**
   * 
   */
  private static final long serialVersionUID = -4198303565500469474L;

  public StrictDirectedNode() {
    super();
  }

  @Override
  public Iterator getRelated() {
    return super.getOutRelated();
  }

}
