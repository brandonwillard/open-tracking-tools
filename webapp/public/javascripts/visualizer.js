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

  for (line_id in lines) {
    if (line_id > 0) {

      var new_marker = new L.Circle(new L.LatLng(
          parseFloat(lines[line_id].originalLat),
          parseFloat(lines[line_id].originalLon)), 10, {
        color : '#00c',
        lat : parseFloat(lines[line_id].kfMeanLat),
        lon : parseFloat(lines[line_id].kfMeanLon)
      });
      group.addLayer(new_marker);

      new_marker.on('click', function(e) {

        overlay.clearLayers();

        var overlay_marker = new L.Circle(new L.LatLng(e.target.options.lat,
            e.target.options.lon), 10, {
          color : '#0c0'
        });
        overlay.addLayer(overlay_marker);

      });
    }
  }
}

function moveMarker() {
  if (i != $("#slider").slider("option", "value"))
    $("#slider").slider("option", "value", i);

  var done = renderMarker();

  if (!done)
    i++;
}

function renderMarker() {
  if (i >= 0 && i < lines.length) {
    group.clearLayers();
//    overlay.clearLayers();

    var isOnEdge = false;
    
    var color = 'green';
    if (lines[i].graphSegmentIds.length > 0) {
      isOnEdge = true;
      color = 'red';
    }
    
    var majorAxis = new L.Polyline([
        new L.LatLng(parseFloat(lines[i].kfMeanLat),
            parseFloat(lines[i].kfMeanLon)),
        new L.LatLng(parseFloat(lines[i].kfMajorLat),
            parseFloat(lines[i].kfMajorLon)) ], {
      fill : true,
      color : '#c00'
    })

    group.addLayer(majorAxis);

    var minorAxis = new L.Polyline([
        new L.LatLng(parseFloat(lines[i].kfMeanLat),
            parseFloat(lines[i].kfMeanLon)),
        new L.LatLng(parseFloat(lines[i].kfMinorLat),
            parseFloat(lines[i].kfMinorLon)) ], {
      fill : true,
      color : '#c0c'
    });

    group.addLayer(minorAxis);
    
    var mean = new L.Circle(new L.LatLng(parseFloat(lines[i].kfMeanLat),
        parseFloat(lines[i].kfMeanLon)), 10, {
      fill : true,
      color : color
    });
    
    group.addLayer(mean);

    var obs = new L.Circle(new L.LatLng(parseFloat(lines[i].originalLat),
        parseFloat(lines[i].originalLon)), 10, {
      fill : true,
      color : '#00c'
    });
    group.addLayer(obs);

//    map.panTo(new L.LatLng(parseFloat(lines[i].originalLat),
//        parseFloat(lines[i].originalLon)));
    map.panTo(new L.LatLng(parseFloat(lines[i].kfMeanLat),
        parseFloat(lines[i].kfMeanLon)));

    renderGraph();

    $("#count_display").html(lines[i].time + ' (' + i + ')');
    return false;
  } else {
    clearInterval(interval);
    return true;
  }
}

function drawEdge(id) {
  $.get('/api/segment', {
    segmentId : id
  }, function(data) {

    var color = "red";
    var geojson = new L.GeoJSON();

    geojson.on('featureparse', function(e) {
      e.layer.setStyle({
        color : e.properties.color,
        weight : 7,
        opacity : 0.4
      });
      if (e.properties && e.properties.popupContent){
        e.layer.bindPopup(e.properties.popupContent);
      } 
    });

//    var escName = data.name.replace(/([\\<\\>'])/g, "\\$1").replace(/\0/g, "\\0");
    var escName = data.name.replace(/([\\<\\>'])/g, "");
    
    data.geom.properties = {
      popupContent: escName,
      color : color
    };

    geojson.addGeoJSON(data.geom);
    overlay.addLayer(geojson);

  });
}

function renderGraph() {
  for ( var j in lines[i].graphSegmentIds) {
    var segment = lines[i].graphSegmentIds[j];

    if (segment.length == 2 && segment[0] > -1) {

      $.get('/api/segment', {
        segmentId : segment[0]
      }, function(data) {

        var color;

        var avg_velocity = Math.abs(segment[1]);

        if (avg_velocity < MAX_SPEED)
          color = '#' + getColor(avg_velocity / MAX_SPEED);
        else
          color = 'purple';

        var geojson = new L.GeoJSON();

        geojson.on('featureparse', function(e) {
          e.layer.setStyle({
            color : e.properties.color,
            weight : 7,
            opacity : 0.3
          });
        });

        data.geom.properties = {
          color : color
        };

        geojson.addGeoJSON(data.geom);
        overlay.addLayer(geojson);

      });
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
