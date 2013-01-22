package org.opentrackingtools.graph_builder;

import org.opentripplanner.graph_builder.impl.osm.OSMPlainStreetEdgeFactory;
import org.opentripplanner.openstreetmap.model.OSMNode;
import org.opentripplanner.openstreetmap.model.OSMWithTags;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetTraversalPermission;
import org.opentripplanner.routing.vertextype.IntersectionVertex;

import com.vividsolutions.jts.geom.LineString;

public class PlainStreetEdgeFactory implements
    OSMPlainStreetEdgeFactory {

  @Override
  public PlainStreetEdge createEdge(OSMNode fromNode,
    OSMNode toNode, OSMWithTags wayOrArea,
    IntersectionVertex startEndpoint,
    IntersectionVertex endEndpoint, LineString geometry,
    String name, double length,
    StreetTraversalPermission permissions, boolean back) {
    return new PlainStreetEdgeWithOSMData(
        wayOrArea.getId(), fromNode.getId(),
        toNode.getId(), startEndpoint, endEndpoint,
        geometry, name, length, permissions, back);

  }

}
