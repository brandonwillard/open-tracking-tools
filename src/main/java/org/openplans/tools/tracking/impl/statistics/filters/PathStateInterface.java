package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Vector;

import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;

public interface PathStateInterface {

  PathEdge getEdge();

  Vector getState();

  InferredPath getPath();

}
