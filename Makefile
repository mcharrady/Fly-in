PY	= python3
MAIN	= main.py
MAP	= maps/easy/01_linear_path.txt

run:
	$(PY) $(MAIN) $(MAP)

install:
	$(PY) -m pip install --upgrade pip
	$(PY) -m pip install mypy flake8

debug:
	$(PY) -m pdb $(MAIN) $(MAP)

clean:
	rm -rf __pycache__
	rm -rf .mypy_cache

lint:
	flake8 .
	mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs

lint-strict:
	flake8 .
	mypy . --strict