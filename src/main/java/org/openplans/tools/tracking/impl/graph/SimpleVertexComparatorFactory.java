package org.openplans.tools.tracking.impl.graph;

import java.io.Serializable;
import java.util.Comparator;
import java.util.List;

import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.graph.VertexComparatorFactory;

public class SimpleVertexComparatorFactory implements VertexComparatorFactory, Serializable {
	private static final long serialVersionUID = 1L;

	@Override
	public Comparator<? super Vertex> getComparator(List<Vertex> vertexById) {
		return new SimpleVertexComparator();
	}

}
