window.onload = function(){
  var main = document.createElement("div");
  main.className = "depreciation-msg"
  var head = document.createElement("div");
  head.className = "depreciation-msg-head"
  main.appendChild(head);
  var body = document.createElement("div");
  body.className = "depreciation-msg-body"
  main.appendChild(body);
  var title = document.createElement("p");
  title.className = "depreciation-msg-title"
  head.appendChild(title);
  var title_text = document.createTextNode("Depreciated!");
  title.appendChild(title_text);
  var desc = document.createElement("div");
  desc.className = "depreciation-msg-desc"
  body.appendChild(desc);
  var desc_1 = document.createTextNode("This version of the docs is outdated. Please visit");
  desc.appendChild(desc_1);
  var desc_2 = document.createElement("a");
  main.className = "depreciation-msg"
  var url = window.location.toString();
  var new_url = url.replace('/projects/api/en/docs_python_api/', '/projects/api/en/gen2_develop/');
  desc_2.href = new_url
  desc.appendChild(desc_2);
  var desc_2_text = document.createTextNode("latest version");
  desc_2.appendChild(desc_2_text);
  var desc_3 = document.createTextNode(" to view latest version of this document");
  desc.appendChild(desc_3);

  var element = document.getElementsByClassName("wy-side-nav-search")[0];
  element.appendChild(main);
}