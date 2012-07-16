/* This program is free software: you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public License
   as published by the Free Software Foundation, either version 3 of
   the License, or (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */

var dataUrl = "/api/traces?vehicleId=";
var coordUrl = "/api/convertToLatLon?";
var startLatLng = new L.LatLng(10.3181373, 123.8956844);

var map;

var vertexLayer = null, edgeLayer = null;

var cloudmadeUrl = 'http://{s}.tiles.mapbox.com/v3/mapbox.mapbox-streets/{z}/{x}/{y}.png';
var cloudmadeAttrib = 'Map data &copy; 2011 OpenStreetMap contributors, Imagery &copy; 2011 CloudMade';
var cloudmadeOptions = {
  maxZoom : 17,
  attribution : cloudmadeAttrib
};

var pointsGroup = new L.LayerGroup();
var edgeGroup = new L.LayerGroup();
var inferredGroup = new L.LayerGroup();
var actualGroup = new L.LayerGroup();
var evaluatedGroup = new L.LayerGroup();
var addedGroup = new L.LayerGroup();
var allInfEdgesGroup = new L.LayerGroup();
var allInfMeansGroup = new L.LayerGroup();

var lines = null;
var i = 0;

var interval = null;
var paths = null;

var MAX_SPEED = 10; // in m/s

$(document).ready(function() {

  map = new L.Map('map');

  var cloudmade = new L.TileLayer(cloudmadeUrl, cloudmadeOptions);
  map.addLayer(cloudmade);
  map.addLayer(pointsGroup);
  map.addLayer(edgeGroup);
  map.addLayer(inferredGroup);
  map.addLayer(actualGroup);
  map.addLayer(evaluatedGroup);
  map.addLayer(addedGroup);
  map.addLayer(allInfEdgesGroup);
  map.addLayer(allInfMeansGroup);

  map.setView(startLatLng, 17, true);

  var layersControl = new L.Control.Layers();
  layersControl.addBaseLayer(cloudmade, "base");
  layersControl.addOverlay(pointsGroup, "points");
  layersControl.addOverlay(edgeGroup, "edges");
  layersControl.addOverlay(inferredGroup, "inferred");
  layersControl.addOverlay(actualGroup, "actual");
  layersControl.addOverlay(evaluatedGroup, "evaluated");
  layersControl.addOverlay(addedGroup, "user added");
  layersControl.addOverlay(allInfEdgesGroup, "all inf. edges");
  layersControl.addOverlay(allInfMeansGroup, "all inf. means");

  map.addControl(layersControl);

  $("#controls").hide();
  $("#pause").hide();

  /*
   * Set default open tab
   */
  var index = $('#filterControls li a').index($('a[href="#tabs-2"]').get(0));
  $('#filterControls').tabs({
    selected : index
  });

  $("#loadDataLink").click(loadData);
  showParticlesMeans();
  showParticlesEdges();
  $("#next").click(nextPoint);
  $("#prev").click(prevPoint);
  $("#play").click(playData);
  $("#pause").click(pauseData);
  $("#playData").click(playData);
  $(".ui-accordion-container").accordion({
    active : "a.default",
    header : "a.accordion-label",
    fillSpace : true,
    alwaysOpen : false
  });
  $("#filterControls").tabs();

  $("#addCoordinates").click(addCoordinates);
  $("#addEdge").click(addEdge);

});

function showParticlesEdges() {
  $("#postParticleEdges").hover(function() {
    var markers = $(this).data("particleEdges");
    if (markers) {
      allInfEdgesGroup.addLayer(markers);
      map.invalidateSize();
    }
  }, function() {
    var markers = $(this).data("particleEdges");
    if (markers) {
      allInfEdgesGroup.removeLayer(markers);
      map.invalidateSize();
    }
  });
}

function showParticlesMeans() {
  $("#postParticleMeans").hover(function() {
    var markers = $(this).data("particleMeans");
    if (markers) {
      allInfMeansGroup.addLayer(markers);
      map.invalidateSize();
    }
  }, function() {
    var markers = $(this).data("particleMeans");
    if (markers) {
      allInfMeansGroup.removeLayer(markers);
      map.invalidateSize();
    }
  });
}

function getEdgeFromId(id) {
  var edge = null;
  $.ajax({
    url : '/api/segment?segmentId=' + id,
    dataType : 'json',
    async : false,
    cache : true,
    success : function(data) {
      edge = data;
    }
  });
  return edge;
}

function addEdge() {
  var id = jQuery('#edge_id').val();
  var edges = id.split(',');
  $.each(edges, function(index, value) {
    var edge = getEdgeFromId(value);
    drawEdge(edge, EdgeType.ADDED);
  });
  map.invalidateSize();
}

function drawCoords(lat, lon, popupMessage, pan, justMarker, color, opacity) {
  var latlng = new L.LatLng(parseFloat(lat), parseFloat(lon));
  
  if (!color)
    color = '#0c0';
  if (!opacity)
    opacity = 1;
    
  marker = new L.Circle(latlng, 10, {
    color : color,
    opacity : opacity,
    lat : parseFloat(lat),
    lon : parseFloat(lon),
    weight : 1
  });
  if (popupMessage != null)
    marker.bindPopup(popupMessage);

  if (justMarker)
    return marker;

  pointsGroup.addLayer(marker);

  if (pan)
    map.panTo(latlng);

  return marker;
}

function drawProjectedCoords(x, y, popupMessage, pan) {
  var latLon = convertToLatLon(new Proj4js.Point(x, y));
  var marker = drawCoords(latLon.lat, latLon.lng, popupMessage, pan);
  map.invalidateSize();

  return marker;
}

function addCoordinates() {

  var coordString = jQuery('#coordinate_data').val();
  var coordSplit = coordString.split(",");

  if (coordSplit.length == 2) {
    var coordGetString = "x=" + coordSplit[0] + "&y=" + coordSplit[1];

    $.get(coordUrl + coordGetString, function(data) {

      lat = data['x'];
      lon = data['y'];

      var latlng = new L.LatLng(parseFloat(lat), parseFloat(lon));
      var new_marker = new L.Circle(latlng, 10, {
        color : '#0c0',
        lat : parseFloat(lat),
        lon : parseFloat(lon)
      });
      pointsGroup.addLayer(new_marker);
      addedGroup.addLayer(new_marker);
      map.panTo(latlng);
    });

    map.invalidateSize();
  }

}

function loadData() {

  var vehicleId = jQuery('#vehicle_id').val();
  var thisDataUrl = dataUrl + vehicleId;
  $.get(thisDataUrl, function(data) {

    $("#loadData").hide();

    lines = data;
    $("#controls").show();

    initSlider();

    map.invalidateSize();
  });

}

function initSlider() {
  $("#slider").slider({
    min : 0,
    max : lines.length
  });

  $("#slider").bind("slidechange", function() {
    i = $("#slider").slider("option", "value");
    moveMarker();
  });

  $("#slider").bind("slide", function(event, ui) {
    pauseData();
  });

}

function playData() {
  $("#play").hide();
  $("#pause").show();

  interval = setInterval(moveMarker, 0.8 * 1000);
}

function pauseData() {
  $("#play").show();
  $("#pause").hide();

  clearInterval(interval);
}

function nextPoint() {
  pauseData();

  $("#slider").slider("option", "value", i);
}

function prevPoint() {
  pauseData();

  i = i - 2;

  $("#slider").slider("option", "value", i);
}

function moveMarker() {
  if (i != $("#slider").slider("option", "value"))
    $("#slider").slider("option", "value", i);

  var done = renderMarker();

  if (!done)
    i++;
}

/*
 * Xian 1980 / Gauss-Kruger zone 21 EPSG:2335 isn't saved locally. if we omit
 * this, then there will be an error followed by a query that should obtain this
 * information. TODO what about different zones? how will they be
 * detected/handled?
 */
Proj4js.defs["EPSG:2335"] = "+proj=tmerc +lat_0=0 +lon_0=123 +k=1 +x_0=21500000 +y_0=0 +a=6378140 +b=6356755.288157528 +units=m +no_defs";

var dest = new Proj4js.Proj("EPSG:4326");
var source = new Proj4js.Proj("EPSG:2335");

function convertToLatLon(srcPoint) {
  var point = new Proj4js.Point(srcPoint.x, srcPoint.y);
  Proj4js.transform(source, dest, point);
  return new L.LatLng(point.y, point.x);
}

function drawResults(mean, major, minor, pointType) {

  var color;
  var fill;
  var groupType;
  if (pointType == PointType.INFERRED_FREE) {
    color = 'red';
    fill = false;
    groupType = inferredGroup;
  } else if (pointType == PointType.INFERRED_EDGE) {
    color = 'red';
    fill = true;
    groupType = inferredGroup;
  } else if (pointType == PointType.ACTUAL_FREE) {
    color = 'black';
    fill = false;
    groupType = actualGroup;
  } else if (pointType == PointType.ACTUAL_EDGE) {
    color = 'black';
    fill = true;
    groupType = actualGroup;
  }

  var meanCoords = convertToLatLon(mean);

  if (major && minor) {
    var majorLatLon = convertToLatLon(major);
    var majorAxis = new L.Polyline([ meanCoords, majorLatLon ], {
      fill : true,
      color : '#c00'
    });

    pointsGroup.addLayer(majorAxis);
    groupType.addLayer(majorAxis);

    var minorLatLon = convertToLatLon(minor);
    var minorAxis = new L.Polyline([ meanCoords, minorLatLon ], {
      fill : true,
      color : '#c0c'
    });

    pointsGroup.addLayer(minorAxis);
    groupType.addLayer(minorAxis);
  }

  var mean = new L.Circle(meanCoords, 5, {
    fill : fill,
    color : color,
    opacity : 1.0
  });

  pointsGroup.addLayer(mean);
  groupType.addLayer(mean);

}

PointType = {
  INFERRED_FREE : 0,
  INFERRED_EDGE : 1,
  ACTUAL_FREE : 2,
  ACTUAL_EDGE : 3
};

function renderMarker() {
  if (i >= 0 && i < lines.length) {
    pointsGroup.clearLayers();
    edgeGroup.clearLayers();
    actualGroup.clearLayers();
    inferredGroup.clearLayers();
    addedGroup.clearLayers();
    evaluatedGroup.clearLayers();
    allInfEdgesGroup.clearLayers();
    allInfMeansGroup.clearLayers();

    /*
     * Draw lines first
     */
    renderGraph();

    /*
     * Show particle cloud and paths
     */
    if (lines[i].infResults) {
      var edgeMarkers = $("#postParticleEdges").data("particleEdges");
      if (edgeMarkers) {
        allInfEdgesGroup.addLayer(edgeMarkers);
        map.invalidateSize();
      }
      var markers = $("#postParticleMeans").data("particleMeans");
      if (markers) {
        allInfMeansGroup.addLayer(markers);
        map.invalidateSize();
      }
    }

    if (lines[i].actualResults) {
      var pointType = PointType.ACTUAL_FREE;

      if (lines[i].actualResults.pathSegments.length > 0) {
        pointType = PointType.ACTUAL_EDGE;
      }

      var results = lines[i].actualResults;
      drawResults(results.meanCoords, results.majorAxisCoords,
          results.minorAxisCoords, pointType);
    }

    var obsCoords = new L.LatLng(parseFloat(lines[i].observedCoords.x),
        parseFloat(lines[i].observedCoords.y));
    var obs = new L.Circle(obsCoords, 10, {
      fill : true,
      color : 'grey',
      opacity : 1.0
    });
    pointsGroup.addLayer(obs);

    map.panTo(obsCoords);

    $("#count_display").html(lines[i].time + ' (' + i + ')');
    return false;
  } else {
    clearInterval(interval);
    return true;
  }
}

EdgeType = {
  ACTUAL : 0,
  INFERRED : 1,
  EVALUATED : 2,
  ADDED : 3,
  INFERRED_ALL: 4
};

function drawEdge(edge, edgeType, layerOnly) {

  var result = null;
  var data = edge;
  var velocity = edge.velocity;
  var avg_velocity = Math.abs(velocity);

  var color;
  var weight = 5;
  var opacity = 0.7;

  var groupType;
  if (edgeType == EdgeType.INFERRED) {
    if (avg_velocity != null && avg_velocity < MAX_SPEED)
      color = '#' + getColor(avg_velocity / MAX_SPEED);
    else
      color = "red";

    weight = 10;
    opacity = 0.3;
    groupType = inferredGroup;
  } else if (edgeType == EdgeType.ACTUAL) {
    color = "black";
    weight = 2;
    opacity = 1.0;
    groupType = actualGroup;
  } else if (edgeType == EdgeType.EVALUATED) {
    color = "blue";
    weight = 20;
    opacity = 0.2;
    groupType = evaluatedGroup;
  } else if (edgeType == EdgeType.ADDED) {
    color = "green";
    weight = 20;
    opacity = 0.2;
    groupType = addedGroup;
  }

  var geojson = new L.GeoJSON();
  geojson.on('featureparse', function(e) {
    e.layer.setStyle({
      color : e.properties.color,
      weight : weight,
      opacity : opacity
    });
    if (e.properties && e.properties.popupContent) {
      e.layer.bindPopup(e.properties.popupContent);
    }
  });

  var escName = data.name.replace(/([\\<\\>'])/g, "");

  data.geom.properties = {
    popupContent : escName,
    color : color
  };

  var layers = new Array(geojson);
  geojson.addGeoJSON(data.geom);

  var angle = data.angle;
  if (angle != null) {
    var lonlat = data.geom.coordinates[data.geom.coordinates.length - 1];
    var myicon = new MyIcon();
    var arrowhead = new L.Marker.Compass(new L.LatLng(lonlat[1], lonlat[0]), {
      icon : myicon,
      clickable : false
    });
    arrowhead.setIconAngle(angle);
    layers.push(arrowhead);
  }

  result = new L.LayerGroup(layers);
  if (!layerOnly) {
    // edgeGroup.addLayer(result);
    groupType.addLayer(result);
  }
  map.invalidateSize();

  return result;
}

MyIcon = L.Icon.extend({
  iconUrl : '/public/images/tab_right.png',
  shadowUrl : null,
  shadowSize : null,
  iconAnchor : new L.Point(10, 22)
});

function createMatrixString(matrix) {
  var resStr = new Array();
  // var covLen = matrix.length;
  // var cols = Math.sqrt(covLen);
  $.each(matrix, function(index, data) {
    var tmpStr = parseFloat(data).toFixed(2);
    // if (index == 0)
    // resStr = "[[" + resStr;
    // if (index % cols == 0)
    // resStr = resStr + "],[";
    resStr.push(tmpStr);
  });

  return resStr;
}

function getPathName(pathSegmentIds) {
  // var resStr = new Array();
  // $.each(pathSegmentIds, function(index, data) {
  // resStr.push(data[0]);
  // });

  return paths[arrayHash(pathSegmentIds)];
}

function renderParticles(isPrior) {
  var vehicleId = jQuery('#vehicle_id').val();
  $
      .ajax({
        url : '/api/traceParticleRecord',
        async : false,
        dataType : 'json',
        data : {
          vehicleId : vehicleId,
          recordNumber : i,
          particleNumber : -1,
          withParent : true,
          isPrior : isPrior
        },
        success : function(data) {

          var particleList = null;
          var particleEdgesDiv = null;
          var particleMeansDiv = null;
          var particleParentMeansDiv = null;
          if (!isPrior) {
            particleList = jQuery("#posteriorParticles");
            particleMeansDiv = jQuery("#postParticleMeans");
            particleEdgesDiv = jQuery("#postParticleEdges");
            particleParentMeansDiv = jQuery("#postParticleParentMeans");
          } else {
            particleList = jQuery("#priorParticles");
            particleMeansDiv = jQuery("#priorParticleMeans");
            particleEdgesDiv = jQuery("#priorParticleEdges");
            particleParentMeansDiv = jQuery("#priorParticleParentMeans");
          }

          var particleMeans = new L.LayerGroup();
          particleMeansDiv.data("particleMeans", particleMeans);

          var particleEdges = new L.LayerGroup();
          particleEdgesDiv.data("particleEdges", particleEdges);

          var particleParentMeans = new L.LayerGroup();
          particleParentMeansDiv.data("particleParentMeans",
              particleParentMeans);

          particleList.empty();

          if (data == null)
            return;

          var particleNumber = 0;
          jQuery
              .each(
                  data,
                  function(_, particleData) {

                    var particleMeanLoc = particleData.particle.infResults.meanCoords;
                    var locLinkName = 'particle' + particleNumber + '_mean'
                        + isPrior;
                    var coordPair = particleMeanLoc.x + ',' + particleMeanLoc.y;
                    var locLink = '<a name="'
                        + locLinkName
                        + '" title="'
                        + coordPair
                        + '" style="color : black" href="javascript:void(0)">mean</a>';

                    var edgeDesc = "free";
                    var edgeId = particleData.particle.infResults.inferredEdge.id;
                    if (edgeId != null) {
                      edgeDesc = edgeId + " (" 
                        + parseFloat(particleData.particle.infResults.inferredEdge.length).toFixed(2) + ")";
                    }
                    var edgeLinkName = 'particle' + particleNumber + '_edge'
                        + isPrior;
                    var edgeLink = '<a name="' + edgeLinkName + '" title="'
                        + edgeDesc
                        + '" style="color : black" href="javascript:void(0)">'
                        + edgeDesc + '</a>';

                    var particleDivId = 'particle' + particleNumber;

                    var optionDiv = jQuery('<div>' + ' ('
                        + parseFloat(particleData.weight).toFixed(2) + '), '
                        + locLink + ', ' + edgeLink + '</div>');
                    // optionDiv.attr("value", particleNumber);

                    if (particleData.isBest)
                      optionDiv.css('background', 'yellow');

                    particleList.append('<a class="accordion-label" href="#">'
                        + particleNumber + '</a>');
                    particleList.append(optionDiv);

                    $('#' + particleDivId).click(function() {
                      if (this.className == 'collapser') {
                        if (this.parentNode.classList.contains("collapsed"))
                          this.parentNode.classList.remove("collapsed");
                        else
                          this.parentNode.classList.add("collapsed");
                      }
                    });

                    var particleMeanLatLon = convertToLatLon(particleMeanLoc);
                    createHoverPointLink(locLinkName, particleMeanLatLon);
                    particleMeans.addLayer(drawCoords(particleMeanLatLon.lat,
                        particleMeanLatLon.lng, null, false, true, null, 
                        particleData.weight * particleData.particle.infResults.particleCount));

                    particleEdges.addLayer(renderPath(
                        particleData.particle.infResults.pathSegments,
                        particleData.particle.infResults.pathDirection,
                        EdgeType.INFERRED_ALL, true));

                    var subList = jQuery('<ul><li><div class="subinfo"></div></li></ul>');
                    var collapsedDiv = subList.find(".subinfo");
                    optionDiv.append(subList);

                    var stateMean = createMatrixString(particleData.particle.infResults.stateMean);
                    collapsedDiv.append('<li>state=' + stateMean + '</li>');

                    var stateCov = createMatrixString(particleData.particle.infResults.stateCovariance);
                    collapsedDiv.append('<li>stateCov=' + stateCov + '</li>');

                    var pathName = getPathName(getEdgeIdsFromSegments(particleData.particle.infResults.pathSegments));
                    var pathData = $('#' + pathName).data('path');
                    if (pathData) {
                      var pathLikelihood = parseFloat(
                          pathData.totalLogLikelihood).toFixed(2);
                      collapsedDiv.append('<li>' + pathName + ', '
                          + pathLikelihood + '</li>');
                    }

                    var edge = particleData.particle.infResults.inferredEdge;
                    if (edge != null) {
                      createHoverLineLink(edgeLinkName, edge);
                    }

                    if (particleData.parent) {
                      var parentList = jQuery("<ul></ul>");
                      collapsedDiv.append(parentList);

                      var parentEdgeDesc = "free";
                      var parentEdgeId = particleData.parent.infResults.inferredEdge.id;
                      if (parentEdgeId != null) {
                        parentEdgeDesc = parentEdgeId + " (" 
                          + parseFloat(particleData.parent.infResults.inferredEdge.length).toFixed(2) + ")";
                      }
                      var parentEdgeLinkName = 'parent_particle'
                          + particleNumber + '_edge' + isPrior;
                      var parentEdgeLink = '<a name="'
                          + parentEdgeLinkName
                          + '" title="'
                          + parentEdgeDesc
                          + '" style="color : black" href="javascript:void(0)">'
                          + parentEdgeDesc + '</a>';

                      var parentParticleMeanLoc = particleData.parent.infResults.meanCoords;
                      var parentLocLinkName = 'parent_particle'
                          + particleNumber + '_mean' + isPrior;
                      var parentCoordPair = parentParticleMeanLoc.x + ','
                          + parentParticleMeanLoc.y;
                      var parentLocLink = '<a name="'
                          + parentLocLinkName
                          + '" title="'
                          + parentCoordPair
                          + '" style="color : black" href="javascript:void(0)">mean</a>';
                      parentList.append("<li>Parent:" + parentLocLink + ', '
                          + parentEdgeLink + "</li>");

                      var parentStateMean = createMatrixString(particleData.parent.infResults.stateMean);
                      parentList.append("<li>state=" + parentStateMean
                          + "</li>");

                      var stateCov = createMatrixString(particleData.parent.infResults.stateCovariance);
                      parentList.append('<li>stateCov=' + stateCov + '</li>');

                      var parentPathName = getPathName(getEdgeIdsFromSegments(particleData.parent.infResults.pathSegments));
                      if (parentPathName) {
                        parentList.append('<li>path=' + parentPathName
                            + '</li>');
                      }

                      var parentMeanLatLon = convertToLatLon(parentParticleMeanLoc);
                      createHoverPointLink(parentLocLinkName, parentMeanLatLon);
                      particleParentMeans.addLayer(drawCoords(
                          parentMeanLatLon.lat, parentMeanLatLon.lng, null,
                          null, true));

                      var parentEdge = particleData.parent.infResults.inferredEdge;
                      if (parentEdge != null) {
                        createHoverLineLink(parentEdgeLinkName, parentEdge);
                      }

                    }

                    particleNumber++;
                  });
        }
      });
}

function createHoverLineLink(linkName, edge) {
  if (edge.id < 0)
    return;
  var edgeLinkJName = 'a[name=' + linkName + ']';
  $(edgeLinkJName).data("edge", edge);
  $(edgeLinkJName).hover(function() {
    var localEdge = $(this).data("edge");
    var edge = this.edge;
    if (edge == null) {
      edge = drawEdge(localEdge, EdgeType.INFERRED);
      this.edge = edge;
    } else {
      edgeGroup.addLayer(edge);
      addedGroup.addLayer(edge);
      map.invalidateSize();
    }
  }, function() {
    var edge = this.edge;
    if (edge != null) {
      edgeGroup.removeLayer(edge);
      addedGroup.removeLayer(edge);
      map.invalidateSize();
    }
  });
}

function createHoverPointLink(linkName, latlon) {
  var locLinkJName = 'a[name=' + linkName + ']';
  $(locLinkJName).data("loc", latlon);
  $(locLinkJName).hover(function() {

    var localLoc = $(this).data("loc");
    var marker = this.marker;
    if (marker == null) {
      marker = drawCoords(localLoc.lat, localLoc.lng, null, false, false, 'red');
      this.marker = marker;
    } else {
      pointsGroup.addLayer(marker);
      addedGroup.addLayer(marker);
      map.invalidateSize();
    }
  }, function() {
    var marker = this.marker;
    if (marker != null) {
      pointsGroup.removeLayer(marker);
      addedGroup.removeLayer(marker);
      map.invalidateSize();
    }
  });
}
function arrayHash(array) {
  var prime = 31;
  var result = 1;
  $.each(array, function(index, value) {
    result = prime * result + parseInt(value);
  });
  return result;
}

function getEdgeIdsFromSegments(segments) {
  var edgeIds = new Array();
  $.each(segments, function(index, data) {
    edgeIds.push(data.id);
  });
  return edgeIds;
}

function renderGraph() {
  paths = {};
  if (lines[i].infResults) {
    var pathList = jQuery("#paths");
    pathList.empty();

    var emptyOption = jQuery('<option id="none">none</option>');
    var startPointsOption = jQuery('<option id="startEdges">startEdges</option>');
    var endPointsOption = jQuery('<option id="endEdges">endEdges</option>');
    pathList.append(emptyOption);
    pathList.append(startPointsOption);
    pathList.append(endPointsOption);

    // TODO FIXME must make a separate call to get this
    // info. no longer contained in every particle.

    var vehicleId = jQuery('#vehicle_id').val();
    var evaluatedPaths = null;
    $
        .ajax({
          url : "/api/evaluatedPaths?vehicleId=" + vehicleId + "&recordNumber="
              + i,
          dataType : 'json',
          async : false,
          success : function(data) {
            evaluatedPaths = data;
          }
        });

    if (evaluatedPaths != null && evaluatedPaths.length > 0) {
      // var limit = 500;
      for ( var k in evaluatedPaths) {
        // if (k >= limit)
        // break;

        // FIXME finish
        var pathName = 'path' + k;
        var edgeIds = getEdgeIdsFromSegments(evaluatedPaths[k].pathEdges);
        var pathStr = parseFloat(evaluatedPaths[k].totalDistance).toFixed(2)
            + ", [" + edgeIds.toString() + "]";
        var option = jQuery('<option id=' + pathName + '>(path' + k + ')  '
            + pathStr + '</option>');
        option.attr("value", pathName);
        option.data("path", evaluatedPaths[k]);
        pathList.append(option);
        paths[arrayHash(edgeIds)] = pathName;

      }
    }

    pathList.change(function() {
      // edgeGroup.clearLayers();
      evaluatedGroup.clearLayers();
      // segments
      $("select option:selected")
          .each(
              function() {
                var pathName = this.value;
                if (pathName !== "none" && pathName !== "startEdges"
                    && pathName !== "endEdges") {
                  var path = $('#' + pathName).data('path');
                  renderPath(path.pathEdges, path.totalDistance,
                      EdgeType.EVALUATED);
                } else if (pathName === "startEdges") {
                  var edges = {};
                  $.each(paths, function(key, value) {
                    var path = $('#' + value).data('path');
                    if (path.startEdge != null)
                      edges[path.startEdge] = path.startEdge;
                  });
                  $.each(edges, function(key, value) {
                    evaluatedGroup.addLayer(drawEdge(value, EdgeType.EVALUATED,
                        true));
                  });
                } else if (pathName === "endEdges") {
                  var edges = {};
                  $.each(paths, function(key, value) {
                    var path = $('#' + value).data('path');
                    if (path.endEdge != null)
                      edges[path.endEdge] = path.endEdge;
                  });
                  $.each(edges, function(key, value) {
                    evaluatedGroup.addLayer(drawEdge(value, EdgeType.EVALUATED,
                        true));
                  });
                }
              });
      map.invalidateSize();
    });

  }

  renderParticles(false);
  renderParticles(true);

  if (lines[i].actualResults) {
    renderPath(lines[i].actualResults.pathSegments,
        lines[i].actualResults.pathDirection, EdgeType.ACTUAL);
  }
}

function renderPath(pathSegments, pathDirection, edgeType, layerOnly) {
  var color;
  var weight = 5;
  var opacity = 0.7;

  var groupType;
  if (edgeType == EdgeType.INFERRED) {
    color = "red";
    weight = 10;
    opacity = 0.3;
    groupType = inferredGroup;
  } else if (edgeType == EdgeType.ACTUAL) {
    color = "black";
    weight = 2;
    opacity = 1.0;
    groupType = actualGroup;
  } else if (edgeType == EdgeType.EVALUATED) {
    color = "blue";
    weight = 20;
    opacity = 0.2;
    groupType = evaluatedGroup;
  } else if (edgeType == EdgeType.ADDED) {
    color = "green";
    weight = 20;
    opacity = 0.1;
    groupType = addedGroup;
  } else if (edgeType == EdgeType.INFERRED_ALL) {
    color = "green";
    weight = 10;
    opacity = 0.06;
    groupType = allInfEdgesGroup;
  }

  var segments = $.extend(true, [], pathSegments);
  if (pathDirection < 0) {
    segments.reverse();
  }

  var latLngs = new Array();
  var justIds = new Array();
  for ( var j in segments) {
    var segmentInfo = segments[j];
    var edgeId = segmentInfo.id;

    if (edgeId > -1) {
      if (j > 0 && j % 5 == 0) {
        justIds.push('<br>' + edgeId);
      } else {
        justIds.push(edgeId);
      }

      for ( var k in segmentInfo.geom.coordinates) {
        latLngs.push(new L.LatLng(segmentInfo.geom.coordinates[k][1],
            segmentInfo.geom.coordinates[k][0]));
      }
    }
  }

  var polyline = new L.Polyline(latLngs, {
    color : color,
    weight : weight,
    opacity : opacity
  });

  polyline.bindPopup(justIds.toString());

  if (!layerOnly) {
    // edgeGroup.addLayer(polyline);
    groupType.addLayer(polyline);
    map.invalidateSize();
  }

  return polyline;
}

L.Marker.Compass = L.Marker
    .extend({
      _reset : function() {
        var pos = this._map.latLngToLayerPoint(this._latlng).round();

        L.DomUtil.setPosition(this._icon, pos);
        if (this._shadow) {
          L.DomUtil.setPosition(this._shadow, pos);
        }

        if (this.options.iconAngle) {
          this._icon.style.WebkitTransform = this._icon.style.WebkitTransform
              + ' rotate(' + this.options.iconAngle + 'deg)';
          this._icon.style.MozTransform = 'rotate(' + this.options.iconAngle
              + 'deg)';
          this._icon.style.MsTransform = 'rotate(' + this.options.iconAngle
              + 'deg)';
          this._icon.style.OTransform = 'rotate(' + this.options.iconAngle
              + 'deg)';
        }

        this._icon.style.zIndex = pos.y;
      },

      setIconAngle : function(iconAngle) {

        if (this._map) {
          this._removeIcon();
        }

        this.options.iconAngle = iconAngle;

        if (this._map) {
          this._initIcon();
          this._reset();
        }
      }

    });

function getColor(f) {
  var n = Math.round(100 * f);

  var red = (255 * n) / 100;
  var green = (255 * (100 - n)) / 100;
  var blue = 0;

  var rgb = blue | (green << 8) | (red << 16);

  return rgb.toString(16);
}
