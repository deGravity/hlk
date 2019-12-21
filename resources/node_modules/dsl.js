"use strict";

// This module includes all the code specifications
// for stitch mesh faces
const fs = require('fs');
// Umbilical start and end rows
let StartRows = 10;
let EndRows = 10;

let YarnInStitchNumber = 67;
let CastOnStitchNumber = 67;
let PlainStitchNumber  = 78;
let CableStitchNumber  = 78; 
let PlatingStitchNumber= 78; 
let OutFile = "";
let anchors = new Set();
let mstate  =  new Set();
let last_instr_bed = 'u';
var EmitStyle = {
	IMM: 0,
	PRE: 1,
	MAKE:2,
	POST:3,
};

let emitted_instructions = {"1":[],"2":[],"3":[]}
let last_pass_was_knit = true;

Helpers.prototype.CableStitchNumber = CableStitchNumber;
Helpers.prototype.PlainStitchNumber = PlainStitchNumber;
Helpers.prototype.last_used_carrier = -1;

//figure out output filename based on input js name / other arguments:
if (process.argv.length >= 2) {
	if (process.argv[1].endsWith(".js")) {
		OutFile = process.argv[1].substr(0, process.argv[1].length - 3) + ".k";
	}
	for (let i = 2; i < process.argv.length; ++i) {
		if (process.argv[i].startsWith("out:")) {
			OutFile = process.argv[i].substr(4);
		}
	}
}

if (OutFile === "") {
	//console.log("NOTE: will not write output file.");
} else {
	//console.log("Will write output to '" + OutFile + "'");
}

function isSameBed(b1, b2){
	if((b1 == 'f' || b1 == 'fs') && (b2 == 'f' || b2 == 'fs')) return true;
	if((b1 == 'b' || b1 == 'bs') && (b2 == 'b' || b2 == 'bs')) return true;
	return false;
}

function parseBedNeedle(bn) {
	if (typeof(bn) !== "string") {
		throw new Error("parseBedNeedle must be called with a string : " + typeof(bn));
	}
	let m = bn.match(/^([fb]s?)([-+]?\d+)$/);
	if (m === null) throw new Error("string '" + bn + "' does not look like a needle");
	return {
		bed:m[1],
		needle:parseInt(m[2])
	};
}

function bnToHalf(bn_str) {
	let bn = parseBedNeedle(bn_str);
	if      (bn.bed === 'f')  return 'f' + (2*bn.needle);
	else if (bn.bed === 'fs') return 'f' + (2*bn.needle+1);
	else if (bn.bed === 'b')  return 'b' + (2*bn.needle+1);
	else if (bn.bed === 'bs') return 'b' + (2*bn.needle);
	else throw "don't know how to half-guage the needle '" + bn_str + "'";
}

function Helpers() {
	this.knitout = [];
	this.racking = 0;
	this.needIn = true; 
	this.out(";!knitout-2"); // version
	this.out(";;Carriers: 1 2 3 4 5 6 7 8 9 10"); //carrier header
}

Helpers.prototype.out = function out(str) {
	this.knitout.push(str);
};

Helpers.prototype.write = function write() {
	if (OutFile !== "") {
		fs.writeFileSync(OutFile, this.knitout.join("\n") + "\n");
	}
};


//Global code helpers, not associated with any stitch face
//-------------------------------------------------------------------------------

// Do NOT change these functions or add anything here to create a custom stitch face

//----- primary functions that keep state for inserting and exiting yarn as well as for reordering isntructions:
Helpers.prototype.knit = function knit(d, bn, Carrier, target = EmitStyle.IMM) {
	if(target == EmitStyle.IMM){
		this.pass_commit();
		this.out("knit " + d + " " + bnToHalf(bn) + " " + Carrier);
	}
	else{
		let instr = ("knit " + d + " " + bnToHalf(bn) + " " + Carrier);
		let is_knitting = true;
		let on_bed = parseBedNeedle(bn).bed;
		let pack = {'i':instr, 'p':is_knitting, 'b':on_bed};

		emitted_instructions[target].push(pack);
	}

	// state maintainance:
	this.last_used_carrier = Carrier;
	anchors.delete(bn);
	mstate.add(bn);
};

Helpers.prototype.tuck = function tuck(d, bn, Carrier, target = EmitStyle.IMM) {
	if(target == EmitStyle.IMM){
		this.pass_commit(); // make sure everything is flushed, and directly call this
		this.out("tuck " + d + " " + bnToHalf(bn) + " " + Carrier);
	}
	else{

		let instr = ("tuck " + d + " " + bnToHalf(bn) + " " + Carrier);
		let is_knitting = true;
		let on_bed = parseBedNeedle(bn).bed;
		let pack = {'i':instr, 'p':is_knitting, 'b':on_bed};


		emitted_instructions[target].push(pack);
	}
	anchors.delete(bn);
	mstate.add(bn);
	this.last_used_Carrier = Carrier;
};

Helpers.prototype.miss = function miss(d, bn, Carrier, target = EmitStyle.IMM) {


	if(target == EmitStyle.IMM){
		this.pass_commit(); // make sure everything is flushed, and directly call this
		this.out("miss " + d + " " + bnToHalf(bn) + " " + Carrier);
	}
	else{
		let instr = ("miss " + d + " " + bnToHalf(bn) + " " + Carrier);
		let is_knitting = true;
		let on_bed = parseBedNeedle(bn).bed;
		let pack = {'i':instr, 'p':is_knitting, 'b':on_bed};

		emitted_instructions[target].push(pack);
	}
	this.last_used_Carrier = Carrier;

};

Helpers.prototype.xfer = function xfer(from, to, target = EmitStyle.IMM) {
	if(target == EmitStyle.IMM){
		this.pass_commit();
		this.out("xfer " + bnToHalf(from) + " " + bnToHalf(to));
	}
	else{
		let instr = ("xfer " + bnToHalf(from) + " " + bnToHalf(to));
		let is_knitting = false;
		let on_bed = parseBedNeedle(from).bed; // don't care
		let pack = {'i':instr, 'p':is_knitting, 'b':on_bed};


		emitted_instructions[target].push(pack);
	}
	anchors.delete(from); // what you planned on dropping is gone, don't drop something arbitrary
	mstate.delete(from);
	mstate.add(to);
};

Helpers.prototype.split = function split(d, from, to, Carrier, target = EmitStyle.IMM){
	this.pass_commit();
	this.out("split " + d + " " + bnToHalf(from) + " " + bnToHalf(to)+ " " +Carrier);
	anchors.delete(from);
	mstate.delete(from);
	mstate.add(to);
	this.pass_emit();

};
Helpers.prototype.pass_emit = function pass_emit(){
	// starts a pass, start collecting instructions
	console.assert( emitted_instructions[EmitStyle.PRE].length === 0 &&
		emitted_instructions[EmitStyle.MAKE].length === 0 &&
		emitted_instructions[EmitStyle.POST].length === 0, "Everything should have been committed.");
};

Helpers.prototype.pass_commit = function pass_commit(){
	// ends a pass, commit any left over instructions
	if( emitted_instructions[EmitStyle.PRE].length === 0 &&
		emitted_instructions[EmitStyle.MAKE].length === 0 &&
		emitted_instructions[EmitStyle.POST].length === 0) return;

	for(let i = 0; i < emitted_instructions[EmitStyle.PRE].length; i++){
		this.out(emitted_instructions[EmitStyle.PRE][i].i);
	}
	for(let i = 0; i < emitted_instructions[EmitStyle.MAKE].length; i++){
		this.out(emitted_instructions[EmitStyle.MAKE][i].i);
	}
	for(let i = 0; i < emitted_instructions[EmitStyle.POST].length; i++){
		this.out(emitted_instructions[EmitStyle.POST][i].i);
	}
	emitted_instructions[EmitStyle.PRE].length = 0 ;
	emitted_instructions[EmitStyle.MAKE].length = 0 ;
	emitted_instructions[EmitStyle.POST].length = 0;
};

//-------------------------------------------------------------------------------
//----------- global helpers and misc code---------------------------------------
Helpers.prototype.raw_inst = function raw_inst(str){
	this.out(str);
}

Helpers.prototype.drop_anchors = function drop_anchors(){
	anchors.forEach(function(bn){
		this.drop(bn);
	}, this);
	anchors.clear();
};

Helpers.prototype.bring_carrier_in = function bring_carrier_in(y, bn){

	// bring carrier in such that the next stitch to be made is at bn
	this.pass_commit(); //commit anything before this pass
	this.out("inhook " + y);
	// dance
	let pbn = parseBedNeedle(bn);
	// inserting hook has to hook in on the front bed
	let is_0 = mstate.has('fs'+pbn.needle);
	let is_1 = mstate.has(('fs'+(pbn.needle-2)));
	let is_2 = mstate.has(('fs'+(pbn.needle-4)));
	let is_3 = mstate.has(('fs'+(pbn.needle-3)));
	let is_4 = mstate.has(('fs'+(pbn.needle-1)));
	this.tuck('-', ('fs' + pbn.needle), y);
	this.tuck('-', ('fs' + (pbn.needle-2)), y );
	this.tuck('-', ('fs' + (pbn.needle-4)), y );
	this.tuck('+', ('fs' + (pbn.needle-3)), y );
	this.tuck('+', ('fs' + (pbn.needle-1)), y );

	this.out("releasehook " + y);

	if (!is_0)
		anchors.add( ('fs'+pbn.needle)   );
	if (!is_1)
		anchors.add( 'fs'+(pbn.needle-2) );
	if (!is_2)
		anchors.add( 'fs'+(pbn.needle-4) );
	if (!is_3)
		anchors.add( 'fs'+(pbn.needle-3) );
	if (!is_4)
		anchors.add( 'fs'+(pbn.needle-1) );

};

Helpers.prototype.take_carrier_out = function take_carrier_out(y, bn){
	// take carrier out, the last stitch made was at bn
	this.pass_commit();
	this.out("outhook " + y);
};


// separate stitch number might be needed for a cable pass
Helpers.prototype.set_stitch_number = function set_stitch_number(n){
	this.out("x-stitch-number " + n);
};

// Get opposite needle
Helpers.prototype.opposite = function opposite(bn){
	let pbn = parseBedNeedle(bn);
	if(pbn.bed == 'f') pbn.bed = 'bs';
	else if(pbn.bed == 'fs') pbn.bed = 'b';
	else if(pbn.bed == 'b') pbn.bed = 'fs';
	else if(pbn.bed == 'bs') pbn.bed = 'f';
	return pbn.bed + pbn.needle;
};

//Get direction from to to
Helpers.prototype.get_dir = function get_dir(from, to){
	let f = parseBedNeedle(from);
	let t = parseBedNeedle(to);
	if(f.bed == t.bed && (f.bed == 'f' || f.bed == 'fs')){
		if(f.needle < t.needle) return '+';
		else return '-';
	}
	else if(f.bed == t.bed && (f.bed == 'b' || f.bed =='bs')){
		if(f.needle < t.needle) return '-';
		else return '+';
	}
	else if(f.bed != t.bed && (f.bed == 'f' || f.bed =='fs') && (t.bed == 'b' || t.bed =='bs')){
		return '+';
	}
	else if(f.bed != t.bed && (f.bed == 'b' || f.bed =='bs') && (t.bed == 'f' || t.bed =='fs')){
		return '-';
	}
	else{
		console.assert(false, "cannot get direction from "+from + " to " + to);
	}
};
// consistent independent of yarn direction, change this if this is not what is expected
Helpers.prototype.first_is_left = function first_is_left(bns, dirs){

	let a = parseBedNeedle(bns[0]);
	let b = parseBedNeedle(bns[1]);
	if(a.bed === b.bed){
		if((a.bed === 'f' )){
			if(a.needle < b.needle) return true;
			return false;
		}
		else{
			if(a.needle > b.needle) return true;
			return false;
		}
	}
	else{
		return true;
	}
};

Helpers.prototype.start_tube = function start_tube(dir, bns, Carrier) {
	let front = [];
	let back = [];
	bns.forEach(function(bn_str){
		let bn = parseBedNeedle(bn_str)
		if (bn.bed === 'f') {
			front.push(bn.needle);
		} else if (bn.bed === 'b') {
			back.push(bn.needle);
		} else {
			console.assert("start_tube should only be called with 'f' or 'b' needles.");
		}
	});
	front.sort(function(a, b){return a-b});
	back.sort(function(a, b){return a-b});


	console.assert(front.length !== 0 && back.length !== 0, "should start a tube with at least a stitch on each bed.");

	// tuck pattern to anchor yarn:
	// v   v   v <--
	//   v   v   -->
	//         ^------ first needle to be knit is here
	let n = Math.max(front[front.length-1], back[back.length-1]);
	let toDrop = [];
	let me = this;
	function initTuck(d, bn, c) {
		me.tuck(d, bn, c);
		toDrop.push(bn);
	}
	this.out("x-stitch-number " + YarnInStitchNumber);
	this.out("inhook " + Carrier);
	initTuck('-', 'f' + n, Carrier);
	initTuck('-', 'f' + (n-2), Carrier);
	initTuck('-', 'f' + (n-4), Carrier);
	initTuck('+', 'f' + (n-3), Carrier);
	initTuck('+', 'f' + (n-1), Carrier);
	this.out("releasehook " + Carrier);

	//make list of needles and directions in tube order:
	let sts = [];
	if (dir === 'clockwise') {
		for (let i = front.length-1; i >= 0; --i) {
			sts.push(['-', 'f' + front[i]]);
		}
		for (let i = 0; i < back.length; ++i) {
			sts.push(['+', 'b' + back[i]]);
		}
	} else { console.assert(dir === 'anticlockwise');
		for (let i = back.length-1; i >= 0; --i) {
			sts.push(['-', 'b' + back[i]]);
		}
		for (let i = 0; i < front.length; ++i) {
			sts.push(['+', 'f' + front[i]]);
		}
	}

	//alternating tuck cast on:
	this.out("x-stitch-number " + CastOnStitchNumber);
	sts.forEach(function(dbn, i) {
		if (i%2 == 0) this.knit(dbn[0], dbn[1], Carrier);
	}, this);
	sts.forEach(function(dbn, i) {
		if (i%2 == 1) this.knit(dbn[0], dbn[1], Carrier);
	}, this);

	//drop everything in 'toDrop' that wasn't part of alternating tucks:
	toDrop.forEach(function(bn){
		//WARNING: this might actually drop **TOO MUCH** if the tucked needles overlap other existing stitches
		let idx = bns.indexOf(bn);
		if (idx === -1) {
			this.drop(bn);
		}
	}, this);

	//knit some plain rows:
	this.out("x-stitch-number " + PlainStitchNumber);
	for (let row = 0; row < StartRows; ++row) {
		sts.forEach(function(dbn, i) {
			this.knit(dbn[0], dbn[1], Carrier);
		}, this);
	}

	let first = 0;
	while (first < sts.length && sts[first][1] !== bns[0]) ++first;
	console.assert(first < sts.length, "First stitch from 'bns' should exist in 'sts'.");

	//knit a bit extra to get aligned to the input bns:
	for (let i = 0; i < first; ++i) {
		let st = sts.shift();
		this.knit(st[0], st[1], Carrier);
		sts.push(st);
	}

	//alternating stitches to separate starting tube from knitting:
	this.out("x-stitch-number " + CastOnStitchNumber);
	sts.forEach(function(dbn, i) {
		if (i%2 == 0) this.knit(dbn[0], dbn[1], Carrier);
	}, this);
	sts.forEach(function(dbn, i) {
		if (i%2 == 1) this.knit(dbn[0], dbn[1], Carrier);
	}, this);

	this.out("x-stitch-number " + PlainStitchNumber);
	//this.out("outhook " + Carrier); 
};


/*
Helpers.prototype.start_tube_xyz = function start_tube(dir, bns, Carrier) {
	let front = [];
	let back = [];
	bns.forEach(function(bn_str){
		let bn = parseBedNeedle(bn_str)
		if (bn.bed === 'f') {
			front.push(bn.needle);
		} else if (bn.bed === 'b') {
			back.push(bn.needle);
		} else {
			console.assert("start_tube should only be called with 'f' or 'b' needles.");
		}
	});
	front.sort(function(a, b){return a-b});
	back.sort(function(a, b){return a-b});


	console.assert(front.length !== 0 && back.length !== 0, "should start a tube with at least a stitch on each bed.");

	// tuck pattern to anchor yarn:
	// v   v   v <--
	//   v   v   -->
	//         ^------ first needle to be knit is here
	let n = Math.max(front[front.length-1], back[back.length-1]);
	let toDrop = [];
	let me = this;
	function initTuck(d, bn, c) {
		me.tuck(d, bn, c);
		toDrop.push(bn);
	}
	this.out("x-stitch-number " + YarnInStitchNumber);
	this.out("inhook " + Carrier);
	initTuck('-', 'f' + n, Carrier);
	initTuck('-', 'f' + (n-2), Carrier);
	initTuck('-', 'f' + (n-4), Carrier);
	initTuck('+', 'f' + (n-3), Carrier);
	initTuck('+', 'f' + (n-1), Carrier);
	this.out("releasehook " + Carrier);

	//make list of needles and directions in tube order:
	let sts = [];
	if (dir === 'clockwise') {
		for (let i = front.length-1; i >= 0; --i) {
			sts.push(['-', 'f' + front[i]]);
		}
		for (let i = 0; i < back.length; ++i) {
			sts.push(['+', 'b' + back[i]]);
		}
	} else { console.assert(dir === 'anticlockwise');
		for (let i = back.length-1; i >= 0; --i) {
			sts.push(['-', 'b' + back[i]]);
		}
		for (let i = 0; i < front.length; ++i) {
			sts.push(['+', 'f' + front[i]]);
		}
	}

	//alternating tuck cast on:
	this.out("x-stitch-number " + CastOnStitchNumber);
	sts.forEach(function(dbn, i) {
		if (i%2 == 0) this.knit(dbn[0], dbn[1], Carrier);
	}, this);
	sts.forEach(function(dbn, i) {
		if (i%2 == 1) this.knit(dbn[0], dbn[1], Carrier);
	}, this);

	//drop everything in 'toDrop' that wasn't part of alternating tucks:
	toDrop.forEach(function(bn){
		//WARNING: this might actually drop **TOO MUCH** if the tucked needles overlap other existing stitches
		let idx = bns.indexOf(bn);
		if (idx === -1) {
			this.drop(bn);
		}
	}, this);

	//rows 1-5 stockinette:
	this.out("x-stitch-number " + PlainStitchNumber);
	for (let row = 0; row < 5; ++row) {
		sts.forEach(function(dbn, i) {
			this.knit(dbn[0], dbn[1], Carrier);
		}, this);
	}

	this.out(";eyelet");
	//eyelet row
	let eyelet_offset = 6;
	{
		// xfer front bed stuff
		let last_needle = -1;
		sts.forEach(function(dbn, i) {
			if(i%eyelet_offset===0 && parseBedNeedle(dbn[1]).bed === 'f')
			{

				this.setRacking(dbn[1],this.opposite(dbn[1]));
				this.xfer(dbn[1],this.opposite(dbn[1]));
				last_needle = parseBedNeedle(dbn[1]).needle;
			}
		}, this);

		sts.forEach(function(dbn, i) {
			if(i%eyelet_offset===0 && parseBedNeedle(dbn[1]).bed === 'f')
			{
				this.setRacking(this.opposite(dbn[1]),'f'+(parseBedNeedle(dbn[1]).needle-1));
				this.xfer(this.opposite(dbn[1]),'f'+(parseBedNeedle(dbn[1]).needle-1));
			}
		}, this);


		this.setRacking();
		sts.forEach(function(dbn, i) {
			if(i%eyelet_offset===0 && parseBedNeedle(dbn[1]).bed === 'b')
			{
				this.xfer((dbn[1]), this.opposite(dbn[1]));
				last_needle = parseBedNeedle(dbn[1]).needle;
			}
		}, this);
		this.setRacking('f'+last_needle, 'b'+(last_needle+1));
		sts.forEach(function(dbn, i) {
			if(i%eyelet_offset===0 && parseBedNeedle(dbn[1]).bed === 'b')
			{
				this.xfer(this.opposite(dbn[1]),'b'+(parseBedNeedle(dbn[1]).needle+1));
			}
		}, this);
		this.setRacking();

		sts.forEach(function(dbn, i) {
			if(i%eyelet_offset===0){
				this.tuck((dbn[0] === '+' ? '-':'+'), dbn[1], Carrier);
			}
			else{
				this.knit(dbn[0], dbn[1], Carrier);
			}
		}, this);
	}

	//stockinette 7-15
	for (let row = 7; row < 15; ++row) {
		sts.forEach(function(dbn, i) {
			this.knit(dbn[0], dbn[1], Carrier);
		}, this);
	}
	//negate elastic affect
	for (let row = 15; row < 19; ++row) {
		sts.forEach(function(dbn, i) {
			this.knit(dbn[0], dbn[1], Carrier);
		}, this);
	}

	let first = 0;
	while (first < sts.length && sts[first][1] !== bns[0]) ++first;
	console.assert(first < sts.length, "First stitch from 'bns' should exist in 'sts'.");

	//knit a bit extra to get aligned to the input bns:
	for (let i = 0; i < first; ++i) {
		let st = sts.shift();
		this.knit(st[0], st[1], Carrier);
		sts.push(st);
	}

	//alternating stitches to separate starting tube from knitting:
	this.out("x-stitch-number " + CastOnStitchNumber);
	sts.forEach(function(dbn, i) {
		if (i%2 == 0) this.knit(dbn[0], dbn[1], Carrier);
	}, this);
	sts.forEach(function(dbn, i) {
		if (i%2 == 1) this.knit(dbn[0], dbn[1], Carrier);
	}, this);

	this.out("x-stitch-number " + PlainStitchNumber);
	//this.out("outhook " + Carrier); //only if the yarn changes..
	
	
	
};
*/

Helpers.prototype.end_tube = function end_tube(dir, bns, cs) {
	//d(bn) returns direction to knit on 'bn' given overall tube direction
	function d(bn_str) {
		let bn = parseBedNeedle(bn_str);
		if (bn.bed[0] === 'f') {
			if (dir === 'clockwise') {
				return '-';
			} else { console.assert(dir === 'anticlockwise', "dir is always clockwise or anticlockwise");
				return '+';
			}
		} else { console.assert(bn.bed[0] === 'b', "bed is always f* or b*");
			if (dir === 'clockwise') {
				return '+';
			} else { console.assert(dir === 'anticlockwise', "dir is always clockwise or anticlockwise");
				return '-';
			}
		}
	}
	//NOTE: in this version extracted form vis knit need to keep track
	//of the last yarn, if it isn't the same as the existing yarn, need 
	//to bring carrier in
	//this.bring_carrier_in(cs,bns[0]); 
	//alternating stitches to separate ending tube from knitting:
	this.out("x-stitch-number " + CastOnStitchNumber);
	bns.forEach(function(bn, i) {
		if (i%2 == 0) this.knit(d(bn), bn, cs);
	}, this);
	bns.forEach(function(bn, i) {
		if (i%2 == 1) this.knit(d(bn), bn, cs);
	}, this);

	this.out("x-stitch-number " + PlainStitchNumber);
	for (let row = 0; row < EndRows; ++row) {
		bns.forEach(function(bn, i) {
			this.knit(d(bn), bn, cs);
		}, this);
	}

	this.out("outhook " + cs);

	bns.forEach(function(bn, i) {
		this.drop(bn);
	}, this);

	this.drop_anchors(); 
};

Helpers.prototype.drop = function drop(bn) {
	this.out("drop " + bnToHalf(bn));
	mstate.delete(bn);
	anchors.delete(bn);
};

Helpers.prototype.setRackingValue = function setRackingValue(r){
	console.assert(Math.abs(r) <= 8, "racking value is too large"+r);
	this.racking = r;
	this.out("rack " + this.racking);
};

Helpers.prototype.setRacking = function setRacking(from_str, to_str) {
	let target;
	if (arguments.length === 0) {
		target = 0;
	} else {
		let from = parseBedNeedle(bnToHalf(from_str));
		let to = parseBedNeedle(bnToHalf(to_str));
		if (from.bed === 'f' && to.bed === 'b') {
			target = from.needle - to.needle;
		} else { console.assert(from.bed === 'b' && to.bed === 'f', "racking " + from_str + " " + to_str);
			target = to.needle - from.needle;
		}
		console.assert(Math.abs(target) <= 8, "Racking out of limits?"+from_str+" "+to_str);
	}
	if (this.racking !== target) {
		this.racking = target;
		this.out("rack " + this.racking);
	}
};


Helpers.prototype.xfer_cycle = function xfer_cycle(opts, from, to, xfers) {
	xfers.forEach(function(xf){
		this.setRacking(xf[0], xf[1]);
		this.xfer(xf[0], xf[1]);
	}, this);
};




Helpers.prototype.stash = function stash(from, to) {
	if (from.length !== to.length) throw new Error("from and to arrays should be the same length");
	if (from.length === 0) return;

	this.setRacking(from[0], to[0]);

	for (let i = 0; i < from.length; ++i) {
		this.xfer(from[i], to[i]);
	}

	this.setRacking();
};

Helpers.prototype.unstash = function unstash(from, to) {
	if (from.length !== to.length) throw new Error("from and to arrays should be the same length");
	if (from.length === 0) return;

	this.setRacking(from[0], to[0]);

	for (let i = 0; i < from.length; ++i) {
		this.xfer(from[i], to[i]);
	}

	this.setRacking();
};
//-------------------------------------------------------------------------------
// ATTN:
// To create a custom stitch face, add your code to the end of this chain
// Make sure you have a custom_make(dirs(array of direction chars), bns(array of bedneedles), carrier(yarn carrier string))
// If your stitch config is setup as a basic_type pass make sure you have a custom_post_make(bns(array of bedneedles))
// If your stitch config is setup as a basic_type pass that ignores xfers, make sure you have a custom_pre_xfer
// that does its own xfer planning
//-------------------------------------------------------------------------------
// Knit
Helpers.prototype.knit_make = function knit_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.knit(dirs[0], bns[0], carrier, EmitStyle.MAKE);

	last_instr_bed = parseBedNeedle(bns[0]).bed;
};
Helpers.prototype.knit_pre_xfer = function knit_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.knit_post_make = function knit_post_make(bns) {
	// noop
};

//-------------------------------------------------------------------------------
// Tuck
Helpers.prototype.tuck_make = function tuck_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 1, "Tuck stitch has diff needle count");
	this.tuck(dirs[0], bns[0], carrier, EmitStyle.MAKE);

	last_instr_bed = parseBedNeedle(bns[0]).bed;
};
Helpers.prototype.tuck_pre_xfer = function tuck_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.tuck_post_make = function tuck_post_make(bns) {
	// noop
};

//-------------------------------------------------------------------------------
// Miss
Helpers.prototype.miss_make = function miss_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 1);
	this.miss(dirs[0], bns[0], carrier, EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[0]).bed;
};
Helpers.prototype.miss_pre_xfer = function miss_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.miss_post_make = function miss_post_make(bns) {
	// noop
};

//-------------------------------------------------------------------------------
// Decrease
Helpers.prototype.decrease_make = function decrease_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 1)
	this.knit(dirs[0], bns[0], carrier,EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[0]).bed;
};
Helpers.prototype.decrease_pre_xfer = function decrease_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.decrease_post_make = function decrease_post_make(bns) {
	// noop
};



//-------------------------------------------------------------------------------
// Decrease l
Helpers.prototype.dec_l_make = function dec_l_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 1)
	this.knit(dirs[0], bns[0], carrier,EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[0]).bed;
};
Helpers.prototype.dec_l_pre_xfer = function dec_l_pre_xfer(from, to, step_from, step_to) {
	console.assert(step_from.length === step_to.length);
	let bns  = [];
	for(let i = 1; i < step_from.length; i++){
		if(step_to[i-1] === step_to[i]){
			bns.push(String(step_from[i-1]));
			bns.push(String(step_from[i]));
		}
	}
	for(let i = 0; i  < bns.length; i+=2){

		let a = parseBedNeedle(bns[i]);
		let b = parseBedNeedle(bns[i+1]);

		if(this.first_is_left(bns)){
			if(a.bed === b.bed){
				// move b to a
				let opp_b = this.opposite(bns[i+1]);
				this.setRacking(bns[i+1], opp_b);
				this.xfer(bns[i+1], opp_b);
				this.setRacking(opp_b, bns[i]);
				this.xfer(opp_b, bns[i]);
				this.setRacking();
			}
			// else just use the regular transfer planner
		}
		else{
			if(a.bed === b.bed){
				// move a to b
				let opp_a = this.opposite(bns[i]);
				this.setRacking(bns[i], opp_a);
				this.xfer(bns[i], opp_a);
				this.setRacking(opp_a, bns[i+1]);
				this.xfer(opp_a, bns[i+1]);
				this.setRacking();
			}
			// else just use regular transfer planner
		}
	}

};

Helpers.prototype.dec_l_post_make = function dec_l_post_make(bns) {
	// noop
};


//-------------------------------------------------------------------------------
// Decrease r
Helpers.prototype.dec_r_make = function dec_r_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 1)
	this.knit(dirs[0], bns[0], carrier,EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[0]).bed;
};
Helpers.prototype.dec_r_pre_xfer = function dec_r_pre_xfer(from, to, step_from, step_to) {
	console.assert(step_from.length === step_to.length);
	let bns = [];
	for(let i = 1; i < step_from.length; i++){
		if(step_to[i-1] === step_to[i]){
			console.assert(bns.length === 0);
			bns.push((step_from[i-1]).toString());
			bns.push((step_from[i]).toString());
		}
	}
	console.assert(bns.length === 2);
	//console.log(bns[0]);
	//console.log(bns[1]);
	let a = parseBedNeedle(bns[0]);
	let b = parseBedNeedle(bns[1]);

	console.assert(bns.length === 2);
	if(!this.first_is_left(bns)){
		if(a.bed === b.bed){
			// move b to a
			let opp_b = this.opposite(bns[1]);
			this.setRacking(bns[1], opp_b);
			this.xfer(bns[1], opp_b);
			this.setRacking(opp_b, bns[0]);
			this.xfer(opp_b, bns[0]);
			this.setRacking();
		}
		// else just use the regular transfer planner
	}
	else{
		if(a.bed === b.bed){
			// move a to b
			let opp_a = this.opposite(bns[0]);
			this.setRacking(bns[0], opp_a);
			this.xfer(bns[0], opp_a);
			this.setRacking(opp_a, bns[1]);
			this.xfer(opp_a, bns[1]);
			this.setRacking();
		}
		// else just use regular transfer planner
	}


};

Helpers.prototype.dec_r_post_make = function dec_r_post_make(bns) {
	// noop
};



//-------------------------------------------------------------------------------
// Increase
Helpers.prototype.increase_make = function increase_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.log(bns);
	pbn = parseBedNeedle(bns[1]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 2)
	//console.log("increase " + dirs + " " + bns);
	this.knit(dirs[0], bns[0], carrier, EmitStyle.MAKE);
	this.tuck(dirs[1] == '+' ? '-' : '+', bns[1], carrier,EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[1]).bed;
};
Helpers.prototype.increase_pre_xfer = function increase_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.increase_post_make = function increase_post_make(bns) {
	// noop
};

//-------------------------------------------------------------------------------
// Increase_split
Helpers.prototype.increase_split_make = function increase_split_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 2);
	let pbn = parseBedNeedle(bns[0]);
	let pbn_n = parseBedNeedle(bns[1]);
	last_instr_bed = parseBedNeedle(bns[1]).bed;
	if(pbn.bed === pbn_n.bed){
		pbn.bed = (pbn.bed === 'f' ? 'bs' : 'fs');
		this.split(dirs[0], bns[0], pbn.bed+pbn.needle, carrier);
		this.setRacking(pbn.bed+pbn.needle, bns[1]);
		this.xfer(pbn.bed+pbn.needle, bns[1]);
		this.setRacking();
	}
	else{
		this.setRacking(bns[0], bns[1]);
		this.split(dirs[0], bns[0], bns[1], carrier);
		this.setRacking();

	}
};
Helpers.prototype.increase_split_pre_xfer = function increase_split_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.increase_split_post_make = function increase_split_post_make(bns) {
	// noop
};


// Increase_l (yo)
Helpers.prototype.increase_l_make = function increase_l_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	pbn = parseBedNeedle(bns[1]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}

	console.assert(bns.length == 2)
	let a = parseBedNeedle(bns[0]);
	let b = parseBedNeedle(bns[1]);
	last_instr_bed = parseBedNeedle(bns[1]).bed;
	if( this.first_is_left(bns, dirs) && a.bed === b.bed){

		this.knit(dirs[0], bns[0], carrier, EmitStyle.MAKE);
		let opp = this.opposite(bns[1]);		
		this.setRacking(bns[0],opp);
		this.xfer(bns[0],opp);
		this.setRacking(opp,bns[1]);
		this.xfer(opp, bns[1]);
		this.setRacking();
		this.tuck(dirs[1] == '+' ? '-' : '+', bns[0], carrier,EmitStyle.MAKE);
	}
	else{
		this.knit(dirs[0], bns[0], carrier, EmitStyle.MAKE);
		this.tuck(dirs[1] == '+' ? '-' : '+', bns[1], carrier,EmitStyle.MAKE);
	}
};
Helpers.prototype.increase_l_pre_xfer = function increase_l_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.increase_l_post_make = function increase_l_post_make(bns) {
	// noop
};

//-------------------------------------------------------------------------------
// Increase_r (yo)
Helpers.prototype.increase_r_make = function increase_r_make(dirs, bns, carrier) {

	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	pbn = parseBedNeedle(bns[1]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	console.assert(bns.length == 2)
	let a = parseBedNeedle(bns[0]);
	let b = parseBedNeedle(bns[1]);
	last_instr_bed = parseBedNeedle(bns[1]).bed;
	if( !this.first_is_left(bns, dirs) && a.bed === b.bed){

		this.knit(dirs[0], bns[0], carrier, EmitStyle.MAKE);
		let opp = this.opposite(bns[1]);		
		this.setRacking(bns[0],opp);
		this.xfer(bns[0],opp);
		this.setRacking(opp,bns[1]);
		this.xfer(opp, bns[1]);
		this.setRacking();
		this.tuck(dirs[1] == '+' ? '-' : '+', bns[0], carrier,EmitStyle.MAKE);
	}
	else{
		this.knit(dirs[0], bns[0], carrier, EmitStyle.MAKE);
		this.tuck(dirs[1] == '+' ? '-' : '+', bns[1], carrier,EmitStyle.MAKE);
	}
};

Helpers.prototype.increase_r_pre_xfer = function increase_r_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.increase_r_post_make = function increase_r_post_make(bns) {
	// noop
};


//-------------------------------------------------------------------------------

// Increase split l
Helpers.prototype.increase_split_l_make = function increase_split_l_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 2);
	let pbn = parseBedNeedle(bns[0]);
	let pbn_n = parseBedNeedle(bns[1]);
	last_instr_bed = parseBedNeedle(bns[1]).bed;
	if(pbn.bed === pbn_n.bed){
		let opp0 = this.opposite(bns[0]);
		this.xfer(bns[0], opp0);
		if(this.first_is_left(bns,dirs)){
			this.setRacking(opp0,bns[1]);
			this.split(dirs[0], opp0, bns[1], carrier);
			this.setRacking(opp0,bns[0]);
			this.xfer(opp0, bns[0]);
			this.setRacking();			
		}
		else{
			this.setRacking(opp0,bns[0]);
			this.split(dirs[0], opp0, bns[0], carrier);
			this.setRacking(opp0,bns[1]);
			this.xfer(opp0, bns[1]);
			this.setRacking();
		}
	}
	else{

		this.setRacking(bns[0], bns[1]);
		this.split(dirs[0], bns[0], bns[1], carrier);
		this.setRacking();

	}
};
Helpers.prototype.increase_split_l_pre_xfer = function increase_split_l_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.increase_split_l_post_make = function increase_split_l_post_make(bns) {
	// noop
};


//-------------------------------------------------------------------------------

// Increase split r
Helpers.prototype.increase_split_r_make = function increase_split_r_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 2);
	let pbn = parseBedNeedle(bns[0]);
	let pbn_n = parseBedNeedle(bns[1]);
	last_instr_bed = parseBedNeedle(bns[1]).bed;
	if(pbn.bed === pbn_n.bed){
		let opp0 = this.opposite(bns[0]);
		this.xfer(bns[0], opp0);
		if(!this.first_is_left(bns,dirs)){
			this.setRacking(opp0,bns[1]);
			this.split(dirs[0], opp0, bns[1], carrier);
			this.setRacking(opp0,bns[0]);
			this.xfer(opp0, bns[0]);
			this.setRacking();
		}
		else{
			this.setRacking(opp0,bns[0]);
			this.split(dirs[0], opp0, bns[0], carrier);
			this.setRacking(opp0,bns[1]);
			this.xfer(opp0, bns[1]);
			this.setRacking();
		}
	}
	else{
		this.setRacking(bns[0], bns[1]);
		this.split(dirs[0], bns[0], bns[1], carrier);
		this.setRacking();

	}

};
Helpers.prototype.increase_split_r_pre_xfer = function increase_split_r_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.increase_split_r_post_make = function increase_split_r_post_make(bns) {
	// noop
};


//-------------------------------------------------------------------------------
// Yarn-end
Helpers.prototype.yarnend_make = function yarnend_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1);
	this.knit(dirs[0], bns[0], carrier);
	last_instr_bed = parseBedNeedle(bns[0]).bed;

};
Helpers.prototype.yarnend_pre_xfer = function yarnend_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.yarnend_post_make = function yarnend_post_make(bns) {
	// noop
};


// Shortrow
Helpers.prototype.shortrow_make = function shortrow_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "Tuck stitch has diff needle count");
	this.tuck(dirs[0], bns[0], carrier, EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[0]).bed;

};
Helpers.prototype.shortrow_pre_xfer = function shortrow_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.shortrow_post_make = function shortrow_post_make(bns) {
	// noop
};



//-------------------------------------------------------------------------------
// C-Shortrow, should make slits possible
Helpers.prototype.cshort_make = function cshort_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "cshort stitch has diff needle count");
	this.miss(dirs[0], bns[0], carrier, EmitStyle.MAKE);
	this.miss((dirs[0]=='+'?'-':'+'), bns[0], carrier, EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[0]).bed;


};
Helpers.prototype.cshort_pre_xfer = function cshort_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.cshort_post_make = function cshort_post_make(bns) {
	// noop
};



//-------------------------------------------------------------------------------




// Purl
Helpers.prototype.purl_make = function purl_make(dirs, bns, carrier) {
	console.assert(bns.length == 1)
	let bn = bns[0];
	let d  = dirs[0];
	let pbn = parseBedNeedle(bn);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	last_instr_bed = pbn.bed;
	if (pbn.bed === 'f'){
		// xfer to back bed
		let from = pbn;
		let to  = JSON.parse(JSON.stringify(from));
		to.bed = 'bs';
		this.xfer(from.bed+from.needle, to.bed+to.needle, EmitStyle.PRE);
		// knit
		this.knit(d, to.bed+to.needle, carrier, EmitStyle.MAKE);
		// xfer to front bed
		this.xfer(to.bed + to.needle, from.bed+from.needle, EmitStyle.POST);
	}
	else if(pbn.bed === 'b'){
		// xfer to front bed
		let from = pbn;
		let to  = JSON.parse(JSON.stringify(from));
		to.bed = 'fs';
		this.xfer(from.bed+from.needle, to.bed+to.needle, EmitStyle.PRE);
		// knit
		this.knit(d, to.bed+to.needle, carrier, EmitStyle.MAKE);
		// xfer to back bed
		this.xfer(to.bed + to.needle, from.bed+from.needle, EmitStyle.POST);
	}
	else{
		console.assert(false, "cannot knit on holding hooks");
	}
};
Helpers.prototype.purl_pre_xfer = function purl_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.purl_post_make = function purl_post_make(bns) {
	// noop
};



//-------------------------------------------------------------------------------
// KnitPlated
Helpers.prototype.knitPlated_make = function knitPlated_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		//console.log('beds different, committing);
		this.pass_commit();
	}
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	let c = carrier.split('').reverse().join(''); // reverse carrier string

	this.knit(dirs[0], bns[0], c, EmitStyle.MAKE);
	last_instr_bed = parseBedNeedle(bns[0]).bed;

};
Helpers.prototype.knitPlated_pre_xfer = function knitPlated_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.knitPlated_post_make = function knitPlated_post_make(bns) {
	// noop
};

//-------------------------------------------------------------------------------
// Cable2
Helpers.prototype.cable2_make = function cable2_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	//todo need to push stitch number also into appropriate list:
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);
	last_instr_bed = parseBedNeedle(bns[0]).bed;


};
Helpers.prototype.cable2_pre_xfer = function cable2_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.cable2_post_make = function cable2_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 2, "cable2 expects only 2 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};

// Cable2b
Helpers.prototype.cable2b_make = function cable2b_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);
	last_instr_bed = parseBedNeedle(bns[0]).bed;


};
Helpers.prototype.cable2b_pre_xfer = function cable2b_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.cable2b_post_make = function cable2b_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 2, "cable2 expects only 2 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);

		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};

// cable 3
//-------------------------------------------------------------------------------

Helpers.prototype.cable3_012_make = function cable3_012_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);
	last_instr_bed = parseBedNeedle(bns[0]).bed;


};

Helpers.prototype.cable3_012_post_make = function cable3_012_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 3, "cable3 expects only 3 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);

		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};


Helpers.prototype.cable3_021_make = function cable3_021_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);


};

Helpers.prototype.cable3_021_post_make = function cable3_021_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 3, "cable3 expects only 3 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);

		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};

Helpers.prototype.cable3_102_make = function cable3_102_make(dirs, bns, carrier) {
	this.pass_commit();
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);


};

Helpers.prototype.cable3_102_post_make = function cable3_102_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 3, "cable3 expects only 3 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);

		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};


Helpers.prototype.cable3_120_make = function cable3_120_make(dirs, bns, carrier) {
	this.pass_commit();
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);


};

Helpers.prototype.cable3_120_post_make = function cable3_120_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 3, "cable3 expects only 3 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);

		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};

Helpers.prototype.cable3_201_make = function cable3_201_make(dirs, bns, carrier) {
	this.pass_commit();
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);


};

Helpers.prototype.cable3_201_post_make = function cable3_201_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 3, "cable3 expects only 3 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);

		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};

Helpers.prototype.cable3_210_make = function cable3_210_make(dirs, bns, carrier) {
	this.pass_commit();
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);


};

Helpers.prototype.cable3_210_post_make = function cable3_210_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 3, "cable3 expects only 3 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.setRacking(this.opposite(bns[0]), bns[1]);
		this.xfer(this.opposite(bns[0]), bns[1]);
		this.setRacking(this.opposite(bns[1]), bns[0]);
		this.xfer(this.opposite(bns[1]), bns[0]);

		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};



// cable 4
//-------------------------------------------------------------------------------
Helpers.prototype.cable4_0123_make = function cable4_0123_make(dirs, bns, carrier) {
	this.pass_commit();
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);


};

Helpers.prototype.cable4_0123_post_make = function cable4_0123_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 4, "cable4 expects only 4 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.xfer(bns[2], this.opposite(bns[2]));
		this.xfer(bns[3], this.opposite(bns[3]));
		this.setRacking(this.opposite(bns[0]), bns[2]);
		this.xfer(this.opposite(bns[0]), bns[2]);
		this.setRacking(this.opposite(bns[1]), bns[3]);
		this.xfer(this.opposite(bns[1]), bns[3]);
		this.setRacking(this.opposite(bns[2]), bns[0]);
		this.xfer(this.opposite(bns[2]), bns[0]);
		this.setRacking(this.opposite(bns[3]), bns[1]);
		this.xfer(this.opposite(bns[3]), bns[1]);
		this.setRacking();

	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};

Helpers.prototype.cable4_2301_make = function cable4_2301_make(dirs, bns, carrier) {
	this.pass_commit();
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	this.out("x-stitch-number " + CableStitchNumber);
	this.knit(dirs[0], bns[0], carrier);


};

Helpers.prototype.cable4_2301_post_make = function cable4_2301_post_make(bns) {

	let b = parseBedNeedle(bns[0]).bed;
	let okay = true;
	for(let i = 1; i < bns.length; i++){
		if(parseBedNeedle(bns[i]).bed != b) okay = false;
	}
	if(okay){
		console.assert(bns.length == 4, "cable4 expects only 4 stitches");
		this.xfer(bns[0], this.opposite(bns[0]));
		this.xfer(bns[1], this.opposite(bns[1]));
		this.xfer(bns[2], this.opposite(bns[2]));
		this.xfer(bns[3], this.opposite(bns[3]));
		this.setRacking(this.opposite(bns[2]), bns[0]);
		this.xfer(this.opposite(bns[2]), bns[0]);
		this.setRacking(this.opposite(bns[3]), bns[1]);
		this.xfer(this.opposite(bns[3]), bns[1]);

		this.setRacking(this.opposite(bns[0]), bns[2]);
		this.xfer(this.opposite(bns[0]), bns[2]);
		this.setRacking(this.opposite(bns[1]), bns[3]);
		this.xfer(this.opposite(bns[1]), bns[3]);

		this.setRacking();


	}
	else{
	}
	this.out("x-stitch-number " + PlainStitchNumber);
};


//-------------------------------------------------------------------------------
// bind-off
Helpers.prototype.bindoff_make = function bindoff_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "Knit stitch has diff needle count");


};
Helpers.prototype.bindoff_pre_xfer = function bindoff_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.bindoff_post_make = function bindoff_post_make(bns) {

	if(bns.length < 1) {
		this.drop(bns[0]);
	}
	else
	{
		let p = bns[0];
		for(let i = 1; i < bns.length; i++){
			let c = bns[i];
			let popp = this.opposite(p);
			if(!isSameBed(parseBedNeedle(p).bed, parseBedNeedle(c).bed)){
				popp = c;	
			}
			this.setRacking(p,popp);
			this.xfer(p,popp);
			if(popp != c){
				this.setRacking(popp, c);
				this.xfer(popp, c);
			}
			this.setRacking();
			let dir = this.get_dir(p,c);
			this.knit(dir, c, this.last_used_carrier);
			p = c;
		}

	}
};

// caston-wrap //eyelets actually
Helpers.prototype.caston_make = function caston_make(dirs, bns, carrier) {
	this.pass_commit();
	console.assert(bns.length == 1, "Knit stitch has diff needle count");
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	this.knit((dirs[0] == '+' ? '-': '+'), bns[0], carrier);


};
Helpers.prototype.caston_pre_xfer = function caston_pre_xfer(from, to, step_from, step_to) {
	// noop
};

Helpers.prototype.caston_post_make = function caston_post_make(bns) {

};

//2-color fair-isle
//-------------------------------------------------------------------------------
Helpers.prototype.fair_a_make = function fair_a_make(dirs, bns, carrier) {
	console.assert(bns.length == 1, "fairisle stitch has diff needle count");

	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}

	last_instr_bed = parseBedNeedle(bns[0]).bed;

	let cs = carrier.split(" ");
	console.assert(cs.length > 1);
	this.knit(dirs[0], bns[0], cs[0] , EmitStyle.MAKE);

};


Helpers.prototype.fair_b_make = function fair_b_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	let cs = carrier.split(" ");
	console.assert(cs.length > 1);
	this.knit(dirs[0], bns[0], cs[1] , EmitStyle.MAKE);

};

Helpers.prototype.fair_at_make = function fair_at_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	let cs = carrier.split(" ");
	console.assert(cs.length > 1);

	this.knit(dirs[0], bns[0], cs[0] , EmitStyle.MAKE);
	this.tuck(dirs[0], bns[0], cs[1] , EmitStyle.POST);
};


Helpers.prototype.fair_bt_make = function fair_bt_make(dirs, bns, carrier) {
	let pbn = parseBedNeedle(bns[0]);
	if(pbn.bed != last_instr_bed){
		this.pass_commit();
	}
	last_instr_bed = parseBedNeedle(bns[0]).bed;
	let cs = carrier.split(" ");
	console.assert(cs.length > 1);

	this.knit(dirs[0], bns[0], cs[1] , EmitStyle.MAKE);
	this.tuck(dirs[0], bns[0], cs[0] , EmitStyle.POST);

};



//-------------------------------------------------------------------------------



//Export modules:
module.exports = {Helpers:Helpers};

