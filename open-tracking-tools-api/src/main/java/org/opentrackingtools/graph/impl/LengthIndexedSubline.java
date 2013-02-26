package org.opentrackingtools.graph.impl;

import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

import org.opentrackingtools.graph.edges.InferredEdge;

public class LengthIndexedSubline {

  protected LineSegment line;
  protected InferredEdge parentEdge;
  private LinearLocation startIndex;
  private LinearLocation endIndex;
  
  public LengthIndexedSubline(LineSegment line, InferredEdge infEdge) {
    this.line = line;
    this.parentEdge = infEdge;
    this.startIndex = infEdge.getLocationIndexedLine().indexOf(line.p0);
    this.endIndex = infEdge.getLocationIndexedLine().indexOf(line.p1);
  }

  public LineSegment getLine() {
    return line;
  }

  public InferredEdge getParentEdge() {
    return parentEdge;
  }

  public LinearLocation getStartIndex() {
    return startIndex;
  }

  public LinearLocation getEndIndex() {
    return endIndex;
  }

}
