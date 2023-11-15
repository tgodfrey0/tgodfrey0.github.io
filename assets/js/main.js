var i = 0;
var txt = "Hi, I'm Toby"; /* The text */
var speed = 100; /* The speed/duration of the effect in milliseconds */
setInterval(blink, 4*speed);

function typeWriter() {
  if (i < txt.length+1) {
    document.getElementById("typed").innerHTML = document.getElementById("typed").innerHTML.slice(0,i) + (txt.charAt(i) + "_");
    i++;
    setTimeout(typeWriter, speed);
  }
}

function blink(){
  if(document.getElementById("typed").innerHTML.charAt(document.getElementById("typed").innerHTML.length-1) == "_"){
    document.getElementById("typed").innerHTML = document.getElementById("typed").innerHTML.slice(0, document.getElementById("typed").innerHTML.length-1);
  } else{
    document.getElementById("typed").innerHTML += "_";
  }
}

function ukrEng(){
  if(document.getElementById("ukr").innerHTML === "Я знаю українську мову."){
    document.getElementById("ukr").innerHTML = "I know Ukrainian.";
  } else {
    document.getElementById("ukr").innerHTML = "Я знаю українську мову.";
  }
  document.getElementById("ukr").blur();
}

function rusEng(){
  if(document.getElementById("rus").innerHTML === "Я учил себя русскому языку."){
    document.getElementById("rus").innerHTML = "I've taught myself Russian.";
  } else {
    document.getElementById("rus").innerHTML = "Я учил себя русскому языку.";
  }
  document.getElementById("rus").blur();
}

function manEng(){
  if(document.getElementById("man").innerHTML === "现在我正在自学汉语。"){
    document.getElementById("man").innerHTML = "Now I'm self-studying Mandarin Chinese.";
  } else {
    document.getElementById("man").innerHTML = "现在我正在自学汉语。";
  }
  document.getElementById("man").blur();
}
