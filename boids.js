// Tamanho da janela
var w = window.innerWidth;
var h = window.innerHeight;

// Variáveis dos boids
var boidsNumber = 30;
var boidRadius = 5;
var boidLine = 2;
var boidColors = [ "#ff4343", "#fff856", "#4681ff", "#55ff46", "#ed5dff" ];
var Vmax = 4;
var Vrange = 10;
var vision = 200;
var visionRange = 1.5 * Math.PI;
var normalizeVelocity = false;
var goal = undefined;
var goalRadius = 5;
var goalColor = "#0000ff";
var obstacleRadius = 5;
var obstacleColor = "#ffffff";
var r1 = 1.5;
var r2 = 1;
var r3 = 1;
var r4 = 1;
var r5 = 1;

// Selecionar o canvas e redimensioná-lo
var canvas = document.getElementById("canvas");
var ctx = canvas.getContext('2d');
canvas.width = w;
canvas.height = h;
canvas.style.backgroundColor = "#333";


// Array para guardar os boids
var boids = [];


// Array de obstaculos
var obstacles = [];


// Funções para trabalhar com vetores
// Soma
function sumVector(v1, v2) {
	return { 'x': v1.x + v2.x, 'y': v1.y + v2.y };
}
// Subtração
function subtractVector(v1, v2) {
	return { 'x': v1.x - v2.x, 'y': v1.y - v2.y };
}
// Módulo
function module(v) {
	return Math.sqrt(Math.pow(v.x, 2) + Math.pow(v.y, 2));
}
// Multiplicação por escalar
function scalarMultiplication(v, scalar) {
	return { 'x': v.x * scalar, 'y': v.y * scalar };
}
// Distancia entre dois pontos
function distance(p1, p2) {
	return Math.sqrt( Math.pow((p2.x - p1.x), 2) + Math.pow((p2.y - p1.y), 2) );
}
// Normaliza um vetor
function normalize(v) {
	var m = module(v);
	return scalarMultiplication(v, 1/m);
}


// Função que limita a velocidade dos boids. Se normalizeVelocity for false, a velocidade
// será limitada ao valor máximo. Se normalizeVelocity for true, a velocidade de todos os
// boids será sempre normalizada e igualada ao máximo
function limitVelocity(i) {
	if (!normalizeVelocity) {
		var m = module(boids[i].velocity);
		if (m > Vmax) {
			boids[i].velocity = normalize(boids[i].velocity);
			boids[i].velocity = scalarMultiplication(boids[i].velocity, Vmax);
		}	
	} else {
		boids[i].velocity = normalize(boids[i].velocity);
		boids[i].velocity = scalarMultiplication(boids[i].velocity, Vmax);	
	}
}


// Função que retorna os vizinhos do boid
function neighbors(i) {
	var neigh = [];
	
	for (var b = 0; b < boids.length; b++) {
		if (b != i) {
			var dist = distance(boids[i].position, boids[b].position);
			if (dist < vision)
				neigh.push(b);
		}		
	}
	
	return neigh;
}


// Função que retorna os obstáculos próximos
function closeObstacles(i) {
	var neigh = [];
	
	for (var o = 0; o < obstacles.length; o++) {
		var obs = obstacles[o];
		var dist = distance(boids[i].position, obs);
		if (dist < vision)
			neigh.push(obs);	
	}
	
	return neigh;
}


// Função que faz os boids viajarem infinitamente pela tela
function wrapBoid(i) {
	while (boids[i].position.x > w) boids[i].position.x -= w;
	while (boids[i].position.x < 0) boids[i].position.x += w;
	while (boids[i].position.y > h) boids[i].position.y -= h;
	while (boids[i].position.y < 0) boids[i].position.y += h;
}


// Função que cria 1 boid
function addBoid() {
	var boid = {
		'color': boidColors[Math.floor(Math.random() * 5)],
		'position': {
			'x': Math.floor(Math.random() * (w - 2 * boidRadius) + boidRadius),
			'y': Math.floor(Math.random() * h - 2 * boidRadius + boidRadius)
		},
		'velocity': {
			'x': Math.random() > 0.5 ? Math.floor(Math.random() * Vrange) : -1 * Math.floor(Math.random() * Vrange),
			'y': Math.random() > 0.5 ? Math.floor(Math.random() * Vrange) : -1 * Math.floor(Math.random() * Vrange)	
		}
	};	
	boids.push(boid);
	limitVelocity(boids.length - 1);
	document.getElementById("boids-number").value = boids.length;
}


// Função que destrói 1 boid
function destroyBoid() {
	boids.splice(-1,1);
	document.getElementById("boids-number").value = boids.length;
}


// Função adiciona os boids ao array
function initialisePositions() {
	for (var b = 0; b < boidsNumber; b++) {
		addBoid();
	}
}


// Desenha os boids na tela
function drawBoids() {
	
	// Limpar o canvas
	ctx.clearRect(0,0,w,h);
	
	// Desenhar o goal
	if (goal != undefined) {
		ctx.beginPath();
		ctx.arc(goal.x, goal.y, goalRadius, 0, 2 * Math.PI, true);
		ctx.fillStyle = goalColor;
		ctx.fill();
	}
	
	// Desenhar os obstáculos
	for (var o = 0; o < obstacles.length; o++) {
		ctx.beginPath();
		ctx.arc(obstacles[o].x, obstacles[o].y, obstacleRadius, 0, 2 * Math.PI, true);
		ctx.fillStyle = obstacleColor;
		ctx.fill();
	}
	
	// Desenhar os boids
	for (var b = 0; b < boids.length; b++) {	
		
		var x              = boids[b].position.x;   // x coordinate
		var y              = boids[b].position.y;   // y coordinate
		var radius         = boidRadius;            // Arc radius
		var startAngle     = 0;                     // Starting point on circle
		var endAngle       = 2 * Math.PI;           // End point on circle
		var anticlockwise  = true;                  // clockwise or anticlockwise
		
		// Primeiro: desenhar a linha mostrando a direção do boid
		ctx.beginPath();
		ctx.moveTo(x,y);
		//var v = Math.sqrt(boids[b].vx^2 + boids[b].vy^2);
		//ctx.lineTo( x + (boidLine * boids[b].vx / v) , y + (boidLine * boids[b].vy / v) );
		ctx.lineTo((x + boids[b].velocity.x), (y + boids[b].velocity.y));
		ctx.strokeStyle = "#aaa";
		ctx.stroke();
		
		// Segundo: desenhar o círculo que representa o corpo do boid
		ctx.beginPath();
		ctx.arc(x, y, radius, startAngle, endAngle, anticlockwise);
		ctx.fillStyle = boids[b].color;
		ctx.fill();
	}
}


// Regra 1: Flocking
function rule1(i) {
	
	var v1 = { 'x': 0, 'y': 0 };
	
	var neigh = neighbors(i);
	if (neigh.length > 0) {
		var centerOfMass = { 'x': 0, 'y': 0 };
		for (var j = 0; j < neigh.length; j++) {
			var b = neigh[j];
			centerOfMass = sumVector(centerOfMass, boids[b].position);
		}
		centerOfMass = scalarMultiplication(centerOfMass, (1 / neigh.length));
		
		v1 = subtractVector(centerOfMass, boids[i].position);
	}
	
	v1 = scalarMultiplication(v1, r1);
	return v1;
}


// Regra 2: Collision Avoidance
function rule2(i) {
	
	var v2 = { 'x': 0, 'y': 0 };
	
	var neigh = neighbors(i);
	if (neigh.length > 0) {
		for (var j = 0; j < neigh.length; j++) {
			var b = neigh[j];
			var d = distance(boids[b].position, boids[i].position);
			if (d < 100) {
				var dif = 100 - d;
				var v = subtractVector(boids[b].position, boids[i].position);
				v = normalize(v);
				v = scalarMultiplication(v, dif);
				v2 = subtractVector(v2, v);
			}
		}
	}
	
	v2 = scalarMultiplication(v2, r2);
	return v2;
}

// Regra 3: Velocity Matching
function rule3(i) {
	
	var v3 = { 'x': 0, 'y': 0 };
	
	var neigh = neighbors(i);
	if (neigh.length > 0) {
		for (var j = 0; j < neigh.length; j++) {
			var b = neigh[j];
			v3 = sumVector(v3, boids[b].velocity);
		}
		v3 = scalarMultiplication(v3, (1 / neigh.length));
		v3 = subtractVector(v3, boids[i].velocity);
	}
	
	v3 = scalarMultiplication(v3, r3);
	return v3;
}


// Regra 4: Goal seeking
function rule4(i) {
	
	var v4 = { 'x': 0, 'y': 0 };

	if (goal != undefined) {
		v4 = subtractVector(goal, boids[i].position);
	}
	
	v4 = scalarMultiplication(v4, r4);
	return v4;
}


// Regra 5: Obstacle Avoidance
function rule5(i) {
	
	var v5 = { 'x': 0, 'y': 0 };
	
	var neigh = closeObstacles(i);
	if (neigh.length > 0) {
		for (var j = 0; j < neigh.length; j++) {
			var obs = neigh[j];
			var d = distance(obs, boids[i].position);
			if (d < 100) {
				var dif = 100 - d;
				var v = subtractVector(obs, boids[i].position);
				v = normalize(v);
				v = scalarMultiplication(v, dif);
				v5 = subtractVector(v5, v);
			}
		}
	}
	
	v5 = scalarMultiplication(v5, r5);
	return v5;
}


// Recalcula as posições dos boids em cada update
function moveAllBoidsToNewPositions() {
	for (var b = 0; b < boids.length; b++) {
		
		// Calcular as velocidades impostas por cada regra		
		var v1 = rule1(b);
		var v2 = rule2(b);
		var v3 = rule3(b);
		var v4 = rule4(b);
		var v5 = rule5(b);

		// Calcular as velocidades resultantes
		boids[b].velocity = sumVector(boids[b].velocity, sumVector(v1, sumVector(v2, sumVector(v3, sumVector(v4, v5)))));
		limitVelocity(b);
		
		
		// Mover o boid
		boids[b].position = sumVector(boids[b].position, boids[b].velocity);
		wrapBoid(b);
	}
}


//Funções do menu
function setR1() {
	r1 = document.getElementById("r1").value;
}
function setR2() {
	r2 = document.getElementById("r2").value;
}
function setR3() {
	r3 = document.getElementById("r3").value;
}
function setR4() {
	r4 = document.getElementById("r4").value;
}
function setR5() {
	r5 = document.getElementById("r5").value;
}
function setVision() {
	vision = document.getElementById("vision").value;
}

function clickHandler(e) {
    var totalOffsetX = 0;
    var totalOffsetY = 0;
    var canvasX = 0;
    var canvasY = 0;
    var currentElement = this;

    do{
        totalOffsetX += currentElement.offsetLeft - currentElement.scrollLeft;
        totalOffsetY += currentElement.offsetTop - currentElement.scrollTop;
    }
    while(currentElement = currentElement.offsetParent)

    canvasX = event.pageX - totalOffsetX;
    canvasY = event.pageY - totalOffsetY;

	var clickMode = document.querySelector('input[name="click-mode"]:checked').value;
	if (clickMode == "true") {
		// Criar goal
		goal = { 'x': canvasX, 'y': canvasY };
	} else {
		// Criar obstáculo
		var obstaculo = { 'x': canvasX, 'y': canvasY };
		obstacles.push(obstaculo);
	}
}

// Função que limpa a tela deletando todos os obstaculos e objetivo
function clear() {
	goal = undefined;
	obstacles = [];
}




// Programa principal
document.getElementById("r1").value = r1;
document.getElementById("r2").value = r2;
document.getElementById("r3").value = r3;
document.getElementById("r4").value = r4;
document.getElementById("r5").value = r5;
document.getElementById("vision").value = vision;

document.getElementById("r1set").addEventListener("click", setR1);
document.getElementById("r2set").addEventListener("click", setR2);
document.getElementById("r3set").addEventListener("click", setR3);
document.getElementById("r4set").addEventListener("click", setR4);
document.getElementById("r5set").addEventListener("click", setR5);
document.getElementById("vision-set").addEventListener("click", setVision);
document.getElementById("add-boid").addEventListener("click", addBoid);
document.getElementById("remove-boid").addEventListener("click", destroyBoid);
document.getElementById("clear").addEventListener("click", clear);
canvas.onclick = clickHandler;

initialisePositions();

function update() {
	
	drawBoids();
	moveAllBoidsToNewPositions();
	
	setTimeout(update, 10);
}

update();
