package org.opentrackingtools.graph;

import org.geotools.graph.build.basic.BasicDirectedGraphBuilder;
import org.geotools.graph.structure.Node;

public class StrictDirectedGraphBuilder extends
    BasicDirectedGraphBuilder {

  public StrictDirectedGraphBuilder() {
    super();
  }

  @Override
  public Node buildNode() {
    return (new StrictDirectedNode());
  }

}
