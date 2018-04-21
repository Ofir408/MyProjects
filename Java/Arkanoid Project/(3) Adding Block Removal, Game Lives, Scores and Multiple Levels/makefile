# 208642496
# benshoo1
compile: bin
	find src -name "*.java" > sources.txt
	javac -cp  biuoop-1.4.jar:. -d bin @sources.txt

run:
	java -cp biuoop-1.4.jar:bin Ass5Game

bin:
	mkdir bin

jar:
	jar cfm ass5game.jar Manifest.mf -C bin/ .