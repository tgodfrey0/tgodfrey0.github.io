var i = 0;
var txt = "Hi, I'm Toby"; /* The text */
var speed = 100; /* The speed/duration of the effect in milliseconds */

function typeWriter() {
  if (i < txt.length+1) {
    document.getElementById("typed").innerHTML = document.getElementById("typed").innerHTML.slice(0,i) + (txt.charAt(i) + "_");
    i++;
    setTimeout(typeWriter, speed);
  }
  setInterval(blink, 5*speed);
}

function blink(){
  if(document.getElementById("typed").innerHTML.charAt(document.getElementById("typed").innerHTML.length-1) == "_"){
    document.getElementById("typed").innerHTML = document.getElementById("typed").innerHTML.slice(0, document.getElementById("typed").innerHTML.length-1);
  } else{
    document.getElementById("typed").innerHTML += "_";
  }
}
