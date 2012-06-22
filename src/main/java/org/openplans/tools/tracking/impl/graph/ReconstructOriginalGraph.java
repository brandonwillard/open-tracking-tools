package org.openplans.tools.tracking.impl.graph;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.openplans.tools.tracking.graph_builder.TurnVertexWithOSMData;
import org.opentripplanner.graph_builder.services.GraphBuilder;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;

public class ReconstructOriginalGraph implements GraphBuilder {

	@Override
	public void buildGraph(Graph graph, HashMap<Class<?>, Object> extra) {
		Graph original = new Graph();
		for (Vertex v : graph.getVertices()) {
			if (!(v instanceof TurnVertexWithOSMData)) continue;
			TurnVertexWithOSMData tv = (TurnVertexWithOSMData) v;
			StreetEdge originalEdge = tv.getOriginal();
			Vertex fv = originalEdge.getFromVertex();
			if (original.getVertex(fv.getLabel()) == null) {
				original.addVertex(fv);
			}
		}
		
		BaseGraph base = new BaseGraph(original);
		graph.putService(BaseGraph.class, base);
	}

	@Override
	public List<String> provides() {
		return Arrays.asList("original");
	}

	@Override
	public List<String> getPrerequisites() {
		return Arrays.asList("streets");
	}

	@Override
	public void checkInputs() {
		//nothing to do
	}

}
