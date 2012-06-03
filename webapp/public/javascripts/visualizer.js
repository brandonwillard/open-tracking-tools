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
var coordUrl = "/api/convertEuclidToCoords?";
var startLatLng = new L.LatLng(10.3181373, 123.8956844); // Portland OR

var map;

var vertexLayer = null, edgeLayer = null;

var group = new L.LayerGroup();
var overlay = new L.LayerGroup();

var lines = null;

var i = 0;

var marker1 = null;
var marker2 = null;

var interval = null;

var MAX_SPEED = 600; // in m/min

$(document)
    .ready(
        function() {

          map = new L.Map('map');

          var cloudmadeUrl = 'http://{s}.tiles.mapbox.com/v3/mapbox.mapbox-streets/{z}/{x}/{y}.png'; 
          var cloudmadeAttrib = 'Map data &copy; 2011 OpenStreetMap contributors, Imagery &copy; 2011 CloudMade'; 
          cloudmade = new L.TileLayer(
            cloudmadeUrl, {
              maxZoom : 17,
              attribution : cloudmadeAttrib
          });

          map.setView(startLatLng, 15, true).addLayer(cloudmade);

          $("#controls").hide();
          $("#pause").hide();

          $("#loadDataLink").click(loadData);
          $("#next").click(nextPoint);
          $("#prev").click(prevPoint);
          $("#play").click(playData);
          $("#pause").click(pauseData);
          $("#showData").click(showData);
          $("#playData").click(playData);

          $("#addCoordinates").click(addCoordinates);
          $("#addEdge").click(addEdge);

          map.addLayer(group);
          map.addLayer(overlay);

        });

function addEdge() {
  
  var id = jQuery('#edge_id').val();
  drawEdge(id);
  map.invalidateSize();
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
        group.addLayer(new_marker);
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

function showData() {
  group.clearLayers();

//  for (line_id in lines) {
//    if (line_id > 0) {
//
//      var new_marker = new L.Circle(new L.LatLng(
//          parseFloat(lines[line_id].originalLat),
//          parseFloat(lines[line_id].originalLon)), 10, {
//        color : '#00c',
//        lat : parseFloat(lines[line_id].kfMeanLat),
//        lon : parseFloat(lines[line_id].kfMeanLon)
//      });
//      group.addLayer(new_marker);
//
//      new_marker.on('click', function(e) {
//
//        overlay.clearLayers();
//
//        var overlay_marker = new L.Circle(new L.LatLng(e.target.options.lat,
//            e.target.options.lon), 10, {
//          color : '#0c0'
//        });
//        overlay.addLayer(overlay_marker);
//
//      });
//    }
//  }
}

function moveMarker() {
  if (i != $("#slider").slider("option", "value"))
    $("#slider").slider("option", "value", i);

  var done = renderMarker();

  if (!done)
    i++;
}

function drawResults(mean, major, minor, isOnEdge, isInferred) {

  if (isInferred) {
    var color = 'red';
    if (isOnEdge) {
      color = 'green'
    }
  } else {
    var color = 'yellow';
    if (isOnEdge) {
      color = 'blue'
    }
  }
  
  var meanCoords = new L.LatLng(parseFloat(mean.x),
          parseFloat(mean.y));
  
  var majorAxis = new L.Polyline([
      meanCoords,
      new L.LatLng(parseFloat(major.x),
          parseFloat(major.y)) ], {
    fill : true,
    color : '#c00'
  })

  group.addLayer(majorAxis);

  var minorAxis = new L.Polyline([
      meanCoords,
      new L.LatLng(parseFloat(minor.x),
          parseFloat(minor.y)) ], {
    fill : true,
    color : '#c0c'
  });

  group.addLayer(minorAxis);
  
  var mean = new L.Circle(meanCoords, 5, {
    fill : true,
    color : color
  });
  
  group.addLayer(mean);

}

function renderMarker() {
  if (i >= 0 && i < lines.length) {
    group.clearLayers();
    overlay.clearLayers();
    
    if (lines[i].infResults) {
      var isOnEdge = false;
      
      if (lines[i].infResults.pathSegmentIds.length > 0) {
        isOnEdge = true;
      }
    
      var results = lines[i].infResults;
      drawResults(results.meanCoords, results.majorAxisCoords, results.minorAxisCoords, isOnEdge, true);
    }

    if (lines[i].actualResults) {
      var isOnEdge = false;
      
      if (lines[i].actualResults.pathSegmentIds.length > 0) {
        isOnEdge = true;
      }
    
      var results = lines[i].actualResults;
      drawResults(results.meanCoords, results.majorAxisCoords, results.minorAxisCoords, isOnEdge, false);
    }
  
    var obsCoords = new L.LatLng(parseFloat(lines[i].observedCoords.x),
        parseFloat(lines[i].observedCoords.y));
    var obs = new L.Circle(obsCoords, 10, {
      fill : true,
      color : 'black'
    });
    group.addLayer(obs);
    
    map.panTo(obsCoords);

    renderGraph();

    $("#count_display").html(lines[i].time + ' (' + i + ')');
    return false;
  } else {
    clearInterval(interval);
    return true;
  }
}

function drawEdge(id, velocity, isInferred) {
  $.get('/api/segment', {
    segmentId : id
  }, function(data) {

    var avg_velocity = Math.abs(velocity);

    var color;

    if (isInferred) {
      color = "yellow";
    } else {
      color = "green";
    }
//    if (avg_velocity < MAX_SPEED)
//      color = '#' + getColor(avg_velocity / MAX_SPEED);
//    else
//      color = 'purple';
    
    var geojson = new L.GeoJSON();

    var weight = 5;
    var opacity = 0.7;
    
    if (isInferred) {
      weight = 13;
      opacity = 0.3
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
    overlay.addLayer(geojson);

  });
}

function renderPath(segmentInfo, isInferred) {
  var segment = segmentInfo;

  if (segmentInfo.length == 2 && segmentInfo[0] > -1) {
    $.get('/api/segment', {
      segmentId : segmentInfo[0]
    }, function(data) {
      drawEdge(segmentInfo[0], segmentInfo[1], isInferred);
    });
  }
}

function renderGraph() {
  if (lines[i].actualResults) {
    for ( var j in lines[i].actualResults.pathSegmentIds) {
      renderPath(lines[i].actualResults.pathSegmentIds[j], false);
    }
  }
  if (lines[i].infResults) {
    for ( var j in lines[i].infResults.pathSegmentIds) {
      renderPath(lines[i].infResults.pathSegmentIds[j], true);
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
