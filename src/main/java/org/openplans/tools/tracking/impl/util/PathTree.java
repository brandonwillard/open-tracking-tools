package org.openplans.tools.tracking.impl.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;

public class PathTree {
  HashMap<PathEdge, PathTree> children = new HashMap<PathEdge, PathTree>();
  List<InferredPath> paths = new ArrayList<InferredPath>();
  PathTree parent;
  boolean isLeaf = false;

  public PathTree() {
  }

  public PathTree(PathTree parent) {
    this.parent = parent;
  }

  public PathTree apply(PathEdge edge, InferredPath path) {
    PathTree next = children.get(edge);

    if (next == null) {
      next = new PathTree(this);
      children.put(edge, next);
      paths.add(path);
      return next;
    } else {
      //do not yet add path to the list of paths for the
      //this because we might need to remove those paths
      return next;
    }
  }

  public PathTree get(PathEdge edge) {
    return children.get(edge);
  }

  boolean isLeaf() {
    return isLeaf;
  }

  public void removePath(InferredPath path) {
    PathTree tree = this;
    while (tree != null) {
      tree.paths.remove(path);
      tree = tree.parent;
    }
  }
}
