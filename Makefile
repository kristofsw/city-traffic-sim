GODOT := /Applications/Godot.app/Contents/MacOS/Godot
PROJECT := .

.PHONY: test lint format format-check check run import install-hooks clean

## Run GUT unit tests headlessly
test:
	$(GODOT) --headless --path $(PROJECT) -s addons/gut/gut_cmdln.gd -gdir=res://tests/unit -gexit

## Lint all scripts (excluding vendor code)
lint:
	gdlint scripts/ tests/unit/

## Format all scripts in place
format:
	gdformat scripts/ tests/unit/

## Check formatting without modifying files (CI gate)
format-check:
	gdformat --check scripts/ tests/unit/

## Full quality gate: format check + lint + tests
check: format-check lint test
	@echo "All checks passed."

## Launch the simulation
run:
	$(GODOT) --path $(PROJECT)

## Regenerate the .godot import cache
import:
	$(GODOT) --import --headless --path $(PROJECT)

## Install the git pre-commit hook
install-hooks:
	cp hooks/pre-commit .git/hooks/pre-commit
	chmod +x .git/hooks/pre-commit
	@echo "Pre-commit hook installed."

## Remove the downloaded GUT source if present
clean:
	rm -rf /tmp/gut_src /tmp/gut.zip