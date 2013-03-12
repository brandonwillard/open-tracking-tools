package org.opentrackingtools.graph;

import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LinearLocation;

public class InferenceGraphSegment {

  protected LineSegment line;
  protected InferenceGraphEdge parentEdge;
  private LinearLocation startIndex;
  private LinearLocation endIndex;
  
  public InferenceGraphSegment(LineSegment line, InferenceGraphEdge infEdge) {
    this.line = line;
    this.parentEdge = infEdge;
    this.startIndex = infEdge.getLocationIndexedLine().indexOf(line.p0);
    this.endIndex = infEdge.getLocationIndexedLine().indexOfAfter(line.p1, this.startIndex);
  }

  public LineSegment getLine() {
    return line;
  }

  public InferenceGraphEdge getParentEdge() {
    return parentEdge;
  }

  public LinearLocation getStartIndex() {
    return startIndex;
  }

  public LinearLocation getEndIndex() {
    return endIndex;
  }

}
