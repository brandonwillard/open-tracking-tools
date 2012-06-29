package org.openplans.tools.tracking.graph_builder;

import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.StateEditor;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.edgetype.TurnEdge;
import org.opentripplanner.routing.vertextype.StreetVertex;
import org.opentripplanner.routing.vertextype.TurnVertex;

public class SimpleTurnEdge extends TurnEdge {

  /**
   * 
   */
  private static final long serialVersionUID = 1291375770696761996L;

  public SimpleTurnEdge(TurnVertex fromv, StreetVertex tov) {
    super(fromv, tov);
  }

  @Override
  public State traverse(State s0) {
    final RoutingRequest options = s0.getOptions();
    if (turnRestricted(s0, options)
        && !options.getModes().contains(TraverseMode.WALK)) {
      return null;
    }
    final TraverseMode traverseMode = s0.getNonTransitMode(options);
    if (!((TurnVertex) fromv).canTraverse(options, traverseMode)) {
      return null;
    }

    final StateEditor s1 = s0.edit(this);

    final double speed = options.getSpeed(s0
        .getNonTransitMode(options));
    final double time = (((TurnVertex) fromv)
        .getEffectiveLength(traverseMode) + turnCost / 20.0) / speed;
    final double weight = ((TurnVertex) fromv).computeWeight(
        s0, options, time);
    s1.incrementWalkDistance(((TurnVertex) fromv).getLength());
    s1.incrementTimeInSeconds((int) Math.ceil(time));
    s1.incrementWeight(weight);
    if (s1.weHaveWalkedTooFar(options))
      return null;

    return s1.makeState();
  }

}
