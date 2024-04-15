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

# Catch-all target: route all unknown targets to Sphinx build help
%:
	@$(SPHINXBUILD) --help

clean:
	 @rm -rf ./doctrees ./html
	 @cd $(HTMLDIR) && ls . | grep -v src | xargs rm -rf
	 @echo "Old HTML files removed."

cleanall:clean
	 @read -p "!!! That would remove all files under .\src !!! Are you sure? " -r; \
		if [[ $$REPLY =~ ^[Yy] ]];
			then rm -rf $(APIDOCDIR);
		fi

apidoc:
	@$(SPHINX_APIDOC) --separate --no-toc --output-dir $(APIDOCDIR) ..\..\PyLabware

html:clean
	@$(SPHINXBUILD) -M $@ . "$(BUILDDIR)" $(SPHINXBUILDOPTS) $(O)

# Cleanup & move compiled HTML into the place where Github pages expect them to be
deploy:clean html
	@mv ./html/* $(HTMLDIR)
	@rm $(HTMLDIR)/objects.inv
	@touch $(HTMLDIR)/.nojekyll
	@echo "HTML files deployed to $(HTMLDIR)."