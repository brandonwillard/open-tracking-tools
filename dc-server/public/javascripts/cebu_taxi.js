//Setup a global namespace for our code.
Taxi = Em.Application.create({

  // When everything is loaded.
  ready: function() {

    // Start polling Twitter
    setInterval(function() {
    	Taxi.vehicles.refresh();
    }, 2000);

    // Call the superclass's `ready` method.
    this._super();
  }
});

Taxi.Vehicle = Em.Object.extend();

//An instance of ArrayController which handles collections.
Taxi.vehicles = Em.ArrayController.create({

  // Default collection is an empty array.
  content: [],

  // Simple id-to-model mapping for searches and duplicate checks.
  _idCache: {},

  // Add a Twitter.Tweet instance to this collection.
  // Most of the work is in the built-in `pushObject` method,
  // but this is where we add our simple duplicate checking.
  addVehicle: function(vehicle) {
    // The `id` from Twitter's JSON
    var id = vehicle.get("id");

    // If we don't already have an object with this id, add it.
    if (typeof this._idCache[id] === "undefined") {
      this.pushObject(vehicle);
      this._idCache[id] = vehicle.id;
    }
  },

  // Public method to fetch more data. Get's called in the loop
  // above as well as whenever the `query` variable changes (via
  // an observer).
  refresh: function() {
	
	
	  
    // Poll Twitter
    var self = this;
    var url = "/api/activeTaxis";
    
    
    
    $.getJSON(url, function(data) {
      // Make a model for each result and add it to the collection.
      for (var i = 0; i < data.length; i++) {
        self.addVehicle(Taxi.Vehicle.create(data[i]));
      }
    });
  }
});


var map;

var startLatLng = new L.LatLng(10.3181373, 123.8956844); 

var mbUrl = 'http://{s}.tiles.mapbox.com/v3/openplans.map-g4j0dszr/{z}/{x}/{y}.png';

var cebuUrl = 'http://cebutraffic.org/tiles/{z}/{x}/{y}.png';

var IncidentIcon = L.Icon.extend({
    iconUrl: '/public/images/caraccident.png',
    iconSize: new L.Point(32, 37),
    iconAnchor: new L.Point(16, 37),
    popupAnchor: new L.Point(0, -37)
});

var incidentIcon = new IncidentIcon();

var FloodIcon = L.Icon.extend({
	iconUrl: '/public/images/flood.png',
    iconSize: new L.Point(32, 37),
    iconAnchor: new L.Point(16, 37),
    popupAnchor: new L.Point(0, -37)
});

var floodIcon = new FloodIcon();

var TaxiIcon = L.Icon.extend({
	iconUrl: '/public/images/taxi.png',
    iconSize: new L.Point(32, 37),
    iconAnchor: new L.Point(16, 37),
    popupAnchor: new L.Point(0, -37)
});

var taxiIcon = new TaxiIcon();

var mbAttrib = 'Traffic overlay powered by OpenPlans Vehicle Tracking Tools, Map tiles &copy; Mapbox (terms).';
var mbOptions = {
  maxZoom : 17,
  attribution : mbAttrib
};

// dynamic height management

$(document).ready(sizeContent);
$(window).resize(sizeContent);

function sizeContent() {
  var newHeight = $(window).height() - $("#header").height() + "px";
  $("#map").css("height", newHeight);
}
	
var incidentLayer = new L.LayerGroup();
var indcidentData = new Array();
var incidentMarkers = {}

function loadIncidents()
{
	$.get('/api/alerts', function(data){
		indcidentData = data;
		
		updateIncidents();
	});
}

function updateIncidents()
{
	incidentLayer.clearLayers();
	incidentMarkers = {}
	
	for(var incident in indcidentData)
	{
		var icon = null;
		
		if(indcidentData[incident].type == "incident")
			icon = incidentIcon;
		else if(indcidentData[incident].type == "flood")
			icon = floodIcon;
		else
			continue;

		incidentMarkers[indcidentData[incident].id] = new L.Marker(new L.LatLng(indcidentData[incident].location_lat.toFixed(5), indcidentData[incident].location_lon.toFixed(5)), {icon: icon});
		incidentMarkers[indcidentData[incident].id].bindPopup(indcidentData[incident].description);
		
		
		incidentLayer.addLayer(incidentMarkers[indcidentData[incident].id]);
	}
}


var taxiLayer = new L.LayerGroup();
var taxiData = new Array();
var taxiMarkers = {}

function loadTaxis()
{
	$.get('/api/activeTaxis', function(data){
		taxiData = data;
		
		updateTaxis();
	});
}

/*function updateTaxis()
{
	taxiLayer.clearLayers();
	taxiMarkers = {}
	
	$('#taxi_list').find("li:gt(0)").remove();
	
	for(var taxi in taxiData)
	{
		taxiMarkers[taxiData[taxi].id] = new L.Marker(new L.LatLng(taxiData[taxi].recentLat.toFixed(5), taxiData[taxi].recentLon.toFixed(5)), {icon: taxiIcon});
		taxiMarkers[taxiData[taxi].id].bindPopup('<strong>' + taxiData[taxi].operator.name + '</strong>: ' + taxiData[taxi].driver.driverId);

		taxiLayer.addLayer(taxiMarkers[taxiData[taxi].id]);
		
		$('#taxi_list').append('<li><a class="taxi_item" href="#" data-id="' + taxiData[taxi].id + '">' + taxiData[taxi].operator.name + ': ' + taxiData[taxi].driver.driverId + '</a></li>');

		$('.taxi_item').click(function(event) {
			
			map.setView(taxiMarkers[$(event.target).data('id')].getLatLng(), 17);
			
			$('#taxi_list').find("li").removeClass("active");
			$(event.target).addClass("active");
			
		});
	}
}*/

// main 

$(document).ready(function() {
	
  map = new L.Map('map');

  var mb = new L.TileLayer(mbUrl, mbOptions);
  map.addLayer(mb);

  var cebu = new L.TileLayer(cebuUrl, mbOptions);

  var congestion = new L.TileLayer(cebuUrl, mbOptions);
  
  map.addLayer(taxiLayer);
  
  map.addLayer(incidentLayer);
  
  map.setView(startLatLng, 15, true);
  
  var overlays = {
		  	"Taxis": taxiLayer,
		    "Velocity": cebu,
		    "Congestion": congestion,
		    "Incidents": incidentLayer
  };
 
  var layersControl = new L.Control.Layers(null, overlays);

  map.addControl(layersControl);
  
  loadIncidents();
  //loadTaxis();
  
  //window.setInterval(loadTaxis, 5000);

});



