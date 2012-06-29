package org.openplans.tools.tracking.impl.graph;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Graph.LoadLevel;

public class BaseGraph implements Serializable {
  private static final long serialVersionUID = 1L;

  private Graph graph;

  BaseGraph(Graph graph) {
    this.graph = graph;
  }

  public Graph getBaseGraph() {
    return graph;
  }

  private void readObject(ObjectInputStream in) throws IOException,
      ClassNotFoundException {
    graph = Graph.load(in, LoadLevel.DEBUG);
  }

  private void writeObject(ObjectOutputStream out)
      throws IOException, ClassNotFoundException {
    graph.save(out);
  }

}
