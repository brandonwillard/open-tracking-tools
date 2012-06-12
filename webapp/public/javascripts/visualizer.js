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
var startLatLng = new L.LatLng(10.3181373, 123.8956844); // Portland OR

var map;

var vertexLayer = null, edgeLayer = null;

var cloudmadeUrl = 'http://{s}.tiles.mapbox.com/v3/mapbox.mapbox-streets/{z}/{x}/{y}.png'; 
var cloudmadeAttrib = 'Map data &copy; 2011 OpenStreetMap contributors, Imagery &copy; 2011 CloudMade'; 
var cloudmadeOptions = { maxZoom : 17, attribution : cloudmadeAttrib };

var pointsGroup = new L.LayerGroup();
var edgeGroup = new L.LayerGroup();
var inferredGroup = new L.LayerGroup();
var actualGroup = new L.LayerGroup();
var evaluatedGroup = new L.LayerGroup();
var addedGroup = new L.LayerGroup();

var lines = null;

var i = 0;

var marker1 = null;
var marker2 = null;

var interval = null;

var MAX_SPEED = 10; // in m/s

$(document)
    .ready(
        function() {

          map = new L.Map('map');

          var cloudmade = new L.TileLayer(cloudmadeUrl, cloudmadeOptions);
          map.addLayer(cloudmade);
          map.addLayer(pointsGroup);
          map.addLayer(edgeGroup);
          map.addLayer(inferredGroup);
          map.addLayer(actualGroup);
          map.addLayer(evaluatedGroup);
          map.addLayer(addedGroup);
          
          map.setView(startLatLng, 17, true);
          
          var layersControl = new L.Control.Layers();
          layersControl.addBaseLayer(cloudmade, "base");
          layersControl.addOverlay(pointsGroup, "points");
          layersControl.addOverlay(edgeGroup, "edges");
          layersControl.addOverlay(inferredGroup, "inferred");
          layersControl.addOverlay(actualGroup, "actual");
          layersControl.addOverlay(evaluatedGroup, "evaluated");
          layersControl.addOverlay(addedGroup, "user added");
          
          map.addControl(layersControl);
          

          $("#controls").hide();
          $("#pause").hide();

          $("#loadDataLink").click(loadData);
          $("#next").click(nextPoint);
          $("#prev").click(prevPoint);
          $("#play").click(playData);
          $("#pause").click(pauseData);
          $("#playData").click(playData);

          $("#addCoordinates").click(addCoordinates);
          $("#addEdge").click(addEdge);


        });

function addEdge() {
  
  var id = jQuery('#edge_id').val();
  drawEdge(id, null, EdgeType.ADDED);
  map.invalidateSize();
}
function drawCoordinates(x, y, popupMessage, pan) {
  var coordGetString = "x=" + x + "&y=" + y;
  var marker = null;
  $.ajax({
    url: coordUrl + coordGetString, 
    dataType: 'json',
    async: false,
    success: function(data) { 
      
      lat = data['x'];
      lon = data['y'];
      
      var latlng = new L.LatLng(parseFloat(lat), parseFloat(lon));
      marker = new L.Circle(latlng, 10, {
        color : '#0c0',
        lat : parseFloat(lat),
        lon : parseFloat(lon),
        weight : 1
      });
      $(this).data('marker', marker);
      pointsGroup.addLayer(marker);
      addedGroup.addLayer(marker);
      if (popupMessage != null)
        marker.bindPopup(popupMessage);
      if (pan)
        map.panTo(latlng);
  }});

  map.invalidateSize();
  
  return marker;
}  

function addCoordinates() {
  
  var coordString = jQuery('#coordinate_data').val();
  var coordSplit = coordString.split(",", 2);
  
  if (coordSplit.length == 2) {
    var coordGetString = "x=" + coordSplit[0] + "&y=" + coordSplit[1];
  
    $.get(coordUrl + coordGetString,
      function(data) { 
        
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
        new_marker.bindPopup("test");
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

  interval = setInterval(moveMarker, 1.5*1000);
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

function drawResults(mean, major, minor, pointType) {

  var color;
  var fill;
  var groupType;
  if (pointType == PointType.INFERRED_FREE) {
    color = 'red';
    fill = false;
    groupType = inferredGroup;
  } else if (pointType == PointType.INFERRED_EDGE) {
    color = 'red'
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
  
  var meanCoords = new L.LatLng(parseFloat(mean.x),
          parseFloat(mean.y));
  
  if (major && minor) {
    var majorAxis = new L.Polyline([
        meanCoords,
        new L.LatLng(parseFloat(major.x),
            parseFloat(major.y)) ], {
      fill : true,
      color : '#c00'
    })
  
    pointsGroup.addLayer(majorAxis);
    groupType.addLayer(majorAxis);
    
    var minorAxis = new L.Polyline([
        meanCoords,
        new L.LatLng(parseFloat(minor.x),
            parseFloat(minor.y)) ], {
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
  groupType.addLayer(mean)
  

}

PointType = {
  INFERRED_FREE : 0,
  INFERRED_EDGE : 1,
  ACTUAL_FREE : 2,
  ACTUAL_EDGE : 3
}

function renderMarker() {
  if (i >= 0 && i < lines.length) {
    pointsGroup.clearLayers();
    edgeGroup.clearLayers();
    actualGroup.clearLayers();
    inferredGroup.clearLayers();
    addedGroup.clearLayers();
    evaluatedGroup.clearLayers();
    
    /*
     * Draw lines first
     */
    renderGraph();
    
    if (lines[i].infResults) {
      var pointType = PointType.INFERRED_FREE;
      
      if (lines[i].infResults.pathSegmentIds.length > 0) {
        pointType = PointType.INFERRED_EDGE;
      }
    
      var results = lines[i].infResults;
      drawResults(results.meanCoords, results.majorAxisCoords, results.minorAxisCoords, pointType);
    }

    if (lines[i].actualResults) {
      var pointType = PointType.ACTUAL_FREE;
      
      if (lines[i].actualResults.pathSegmentIds.length > 0) {
        pointType = PointType.ACTUAL_EDGE;
      }
    
      var results = lines[i].actualResults;
      drawResults(results.meanCoords, results.majorAxisCoords, results.minorAxisCoords, pointType);
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
    ADDED : 3
}

function drawEdge(id, velocity, edgeType) {
  
  var geojson = new L.GeoJSON();
  $.ajax({
    url: '/api/segment?segmentId=' + id, 
    dataType: 'json',
    async: false,
    success: function(data) {

    var avg_velocity = Math.abs(velocity);

    var color;
    var weight = 5;
    var opacity = 0.7;

    var groupType;
    if (edgeType == EdgeType.INFERRED) {
      if (avg_velocity != null
          && avg_velocity < MAX_SPEED)
        color = '#' + getColor(avg_velocity / MAX_SPEED);
      else
        color = "red";
      
      weight = 10;
      opacity = 0.3
      groupType = inferredGroup;
    } else if (edgeType == EdgeType.ACTUAL){
      color = "black";
      weight = 2;
      opacity = 1.0;
      groupType = actualGroup;
    } else if (edgeType == EdgeType.EVALUATED){
      color = "yellow";
      weight = 20;
      opacity = 0.2
      groupType = evaluatedGroup;
    } else if (edgeType == EdgeType.ADDED){
      color = "green";
      weight = 20;
      opacity = 0.2
      groupType = addedGroup;
    }
    
    geojson.on('featureparse', function(e) {
      e.layer.setStyle({
        color : e.properties.color,
        weight : weight,
        opacity : opacity
      });
      if (e.properties && e.properties.popupContent){
        e.layer.bindPopup(e.properties.popupContent);
      } 
    });

    var escName = data.name.replace(/([\\<\\>'])/g, "");
    
    data.geom.properties = {
      popupContent: escName,
      color : color
    };

    geojson.addGeoJSON(data.geom);
    edgeGroup.addLayer(geojson);
    groupType.addLayer(geojson);
    map.invalidateSize();

  }});
  
  return geojson;
}


function renderPath(segmentInfo, edgeType) {
  var segment = segmentInfo;

  if (segmentInfo.length == 2 && segmentInfo[0] > -1) {
    $.get('/api/segment', {
      segmentId : segmentInfo[0]
    }, function(data) {
      drawEdge(segmentInfo[0], segmentInfo[1], edgeType);
    });
  }
}

function renderParticles() {
  var vehicleId = jQuery('#vehicle_id').val();
  $.get('/api/particleDetails', {
      vehicleId: vehicleId,
      recordNumber: i
    }, function(data) {
      
      var particleList = jQuery("#particles");
      particleList.empty();

      var particleNumber = 0;
      jQuery.each(data, function(_, particleData) {
        
        var locLinkName = 'particle' + particleNumber + '_mean';
        var coordPair = particleData.meanLoc.x + ',' + particleData.meanLoc.y;
        var locLink = '<a name="' + locLinkName + '" title="' + coordPair 
          + '" style="color : black" href="javascript:void(0)">mean</a>';
        
        var edgeDesc = "free";
        if (particleData.edgeId != null) {
          edgeDesc = particleData.edgeId;
        }
        var edgeLinkName = 'particle' + particleNumber + '_edge'; 
        var edgeLink = '<a name="' + edgeLinkName + '" title="' 
          + edgeDesc + '" style="color : black" href="javascript:void(0)">' + edgeDesc + '</a>';
        
        var particleDivId = 'particle' + particleNumber; 
        var collapser = '<div class="collapser" id="' + particleDivId + '"></div>';
        
        var option = jQuery('<li></li>');
        option.attr("value", particleNumber);
        
        var optionDiv = jQuery('<div class="collapsed"><span class="property">' 
            + particleNumber + ' (' + particleData.weight + '), ' + locLink + ', ' + edgeLink + '</span>' 
            + collapser + '</div>');
        option.append(optionDiv);
        particleList.append(option);
        
        var subList = jQuery('<ul class="obj collapsible"></ul>');
        subList.append('<li><span class="property"><div class="collapsed">test</div></span></li>');
        optionDiv.append(subList);
        
        $('#' + particleDivId).click(function() {
          if (this.className == 'collapser') {
              if (this.parentNode.classList.contains("collapsed"))
                  this.parentNode.classList.remove("collapsed");
              else
                  this.parentNode.classList.add("collapsed");
          }
        });
        
        
        var locLinkJName = 'a[name=' +locLinkName + ']'; 
        $(locLinkJName).hover(function() { 
          var marker = $(locLinkJName).data('marker');
          if (marker == null) {
            marker = drawCoordinates(particleData.meanLoc.x, particleData.meanLoc.y, null, false);
            $(locLinkJName).data('marker', marker);
          } else {
            pointsGroup.addLayer(marker);
            addedGroup.addLayer(marker);
            map.invalidateSize();
          }
        }, function() { 
          var marker = $(locLinkJName).data('marker');
          if (marker != null) {
            pointsGroup.removeLayer(marker);
            addedGroup.removeLayer(marker);
            map.invalidateSize();
          }
        }); 
        
        if (particleData.edgeId != null) {
          var edgeLinkJName = 'a[name=' + edgeLinkName + ']'; 
          $(edgeLinkJName).hover(function() { 
            var edge = $(edgeLinkJName).data('edge');
            if (edge == null) {
              edge = drawEdge(particleData.edgeId, null, EdgeType.ADDED);
              $(edgeLinkJName).data('edge', edge);
            } else {
              edgeGroup.addLayer(edge);
              addedGroup.addLayer(edge);
              map.invalidateSize();
            }
          }, function() { 
            var edge = $(edgeLinkJName).data('edge');
            if (edge != null) {
              edgeGroup.removeLayer(edge);
              addedGroup.removeLayer(edge);
              map.invalidateSize();
            }
          }); 
        }
        
        particleNumber++;
      });
  });
}

function renderGraph() {
  if (lines[i].infResults) {
    
//    if (lines[i].infResults.evaluatedPaths.length > 0) {
//      var evaledPaths = lines[i].infResults.evaluatedPaths;
//      var ids = new Array();
//      for (var k in evaledPaths) {
//        for (var l in evaledPaths[k]) {
//          var idVelPair = new Array(evaledPaths[k][l], 0);
//          renderPath(idVelPair, EdgeType.EVALUATED);
//        }
//      }
//    }
    
    for ( var j in lines[i].infResults.pathSegmentIds) {
      renderPath(lines[i].infResults.pathSegmentIds[j], EdgeType.INFERRED);
    }
  }
  
  renderParticles();
  
  if (lines[i].actualResults) {
    for ( var j in lines[i].actualResults.pathSegmentIds) {
      renderPath(lines[i].actualResults.pathSegmentIds[j], EdgeType.ACTUAL);
    }
  }
}

function getColor(f) {
  var n = Math.round(100 * f);

  var red = (255 * n) / 100
  var green = (255 * (100 - n)) / 100;
  var blue = 0

  var rgb = blue | (green << 8) | (red << 16);

  return rgb.toString(16);
}
