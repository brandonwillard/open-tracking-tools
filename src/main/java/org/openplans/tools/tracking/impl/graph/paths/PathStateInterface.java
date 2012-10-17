package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.Vector;


public interface PathStateInterface {

  PathEdge getEdge();

  Vector getState();

  InferredPath getPath();

}
