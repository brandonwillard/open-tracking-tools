package org.opentrackingtools.graph.impl;

import org.geotools.graph.build.basic.BasicDirectedGraphBuilder;
import org.geotools.graph.structure.Node;

public class StrictDirectedGraphBuilder extends BasicDirectedGraphBuilder {

  public Node buildNode() {
    return(new StrictDirectedNode());
  }
  
  public StrictDirectedGraphBuilder() {
    super();
  }

}
