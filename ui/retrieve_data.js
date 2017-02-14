var fields = {};
var data_class = "data_output";
var ws = null;

function registerFieldHandlers() {
  $("."+data_class).each(function(i, element) {
    var field_name = element.getAttribute("field");
    tryInitField(field_name);
    addHandler(field_name, function() { updateFieldHandler(field_name); });
    fields[field_name].text_fields.push(element);
  });
}

// Adds handler for field_name.
function addHandler(field_name, handler) {
  tryInitField(field_name);
  fields[field_name].handlers.push(handler);
}

// Adds element to fields object and fills it out with blanks.
// Does not do anything if handlers[field] is defined.
// Returns false if nothing done; true if filled with blanks.
function tryInitField(field) {
  if (typeof field !== "string") {
    return false;
  }
  if (fields[field] === undefined) {
    fields[field] = {};
    fields[field].handlers = [];
    fields[field].value = null;
    fields[field].text_fields = [];
    return true;
  } else {
    return false;
  }
}

function updateFieldHandler(field_name) {
  tryInitField(field_name);
  var field = fields[field_name];
  for (var i in field.text_fields) {
    field.text_fields[i].innerHTML = JSON.stringify(field.value);
  }
}

function processSocketReceive(evt) {
  var msg = JSON.parse(evt.data);
  for (var f in msg) {
    tryInitField(f);
    fields[f].value = msg[f];
    for (var i in fields[f].handlers) {
      fields[f].handlers[i](fields[f].value);
    }
  }
}

function sendMessage(queue, content) {
  var request = '{"' + queue + '":' + JSON.stringify(content) + '}';
  ws.send(request);
}

var a = 0;
function sendRequests() {
  if (ws == null || ws.readyState != ws.OPEN) {
    return;
  }
  sendMessage("ping", {a: ++a});
  var request = "{";
  for (var f in fields) {
    if (f === "time") {
      continue;
    }
    if (typeof f === "string") {
      request += '"' + f + '":null,';
    }
  }
  request.slice(0, -1);
  request += "}";
  ws.send(request);
}

function initializeWebsocket() {
  ws = new WebSocket("ws://beaglebone:13000");
  ws.onmessage = processSocketReceive;
  // Avoid excessive polling; wait 1 sec before trying again.
  ws.onclose = function() { setTimeout(initializeWebsocket, 1000); };
}

function periodic(method, timeout) {
  method();
  setTimeout(function() { periodic(method, timeout); }, timeout);
}

// schedule the first invocation:

$(window).on('load', function() {
  if (!("WebSocket" in window)) {
    alert("WebSocket not supported by your browser");
  }
  registerFieldHandlers();
  initializeBoatHandlers();

  initializeWebsocket();
  periodic(sendRequests, 100);
});
