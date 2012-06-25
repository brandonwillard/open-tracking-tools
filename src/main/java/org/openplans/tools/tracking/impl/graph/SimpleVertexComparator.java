package org.openplans.tools.tracking.impl.graph;

import java.util.Comparator;

import org.opentripplanner.routing.graph.Vertex;

public class SimpleVertexComparator implements Comparator<Vertex> {
	public int compare(Vertex v1, Vertex v2) {
		if (v1.getY() == v2.getY()) {
			return (int) Math.signum((v1.getX() - v2.getX()));
		}
		return (int) Math.signum(v1.getY() - v2.getY());
	}
}
