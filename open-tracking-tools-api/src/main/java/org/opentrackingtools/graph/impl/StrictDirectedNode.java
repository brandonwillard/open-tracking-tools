package org.opentrackingtools.graph.impl;

import org.geotools.graph.structure.basic.BasicDirectedNode;

import java.util.Iterator;

/**
 * Simply changes getRelated to return out nodes, so that
 * generic traversal code in geotools will obey direction.
 * @author bwillard
 *
 */
public class StrictDirectedNode extends BasicDirectedNode {

  public StrictDirectedNode() {
    super();
  }

  @Override
  public Iterator getRelated() {
    return super.getOutRelated();
  }

  
}
