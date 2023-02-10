# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXBUILDOPTS		?= "-a"
SPHINXBUILD		?= sphinx-build
SPHINX_APIDOC	?= sphinx-apidoc
APIDOCDIR		=./api
BUILDDIR		= .
HTMLDIR			= ../

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) --help

.PHONY: help Makefile

cleanhtml:
	 @cd $(HTMLDIR) && ls . | grep -v src | xargs rm -rf
	 @echo "Old HTML files removed"

cleanbuild:
	 @rm -rf ./doctrees ./html
	 @echo "Old build tree removed."

cleanall:clean
	 @read -p "!!! That would remove all files under .\src !!! Are you sure? " -r; \
		if [[ $$REPLY =~ ^[Yy] ]];
			then rm -rf $(APIDOCDIR);
		fi

apidoc:
	@$(SPHINX_APIDOC) --separate --no-toc --output $(APIDOCDIR) ..\PyLabware


# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%:cleanbuild
	@$(SPHINXBUILD) -M $@ . "$(BUILDDIR)" $(SPHINXBUILDOPTS) $(O)

# Cleanup & move compiled HTML into the place where Github pages expect them to be
deploy:cleanhtml html
	@mv ./html/* $(HTMLDIR)
	@rm $(HTMLDIR)/objects.inv
	@touch $(HTMLDIR)/.nojekyll
	@echo "HTML files deployed to $(HTMLDIR)."
