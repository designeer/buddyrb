About this blog of JB Park
===========================
URL :  [https://designeer.github.io/buddyrb/](https://designeer.github.io/buddyrb/)  

## Install ruby, jekyll, and bundle on your PC.  
To install ruby refer to [https://rubyinstaller.org/](https://rubyinstaller.org/) for Windows10,
[]() for Ubuntu.

To install jekyll, bundle, and etc. : []() -> for Windows10, []() -> for Ubuntu

## Make a new Github repo for Github pages

https://qiita.com/sota_mikami/items/c6038cf13fd84b519a61


## Adapt Jekyll template theme on your local  

### 1. Fork beautiful-jekyll repository.
 [beautiful-jekyll](https://deanattali.com/beautiful-jekyll/)

### 2. Rename the repository to <yourusername>.github.io

### 3. Launch the website on local

```bash
# C:\Users\layca\workspace\BlogPosting\buddyrb
$ ls
Mode                LastWriteTime         Length Name
----                -------------         ------ ----
d-----       2019/12/27     18:02                .github
d-----       2019/12/27     18:02                css
d-----       2019/12/27     18:02                img
d-----       2019/12/27     18:02                js
d-----       2019/12/27     18:02                _data
d-----       2019/12/27     18:02                _includes
d-----       2019/12/27     18:02                _layouts
d-----       2019/12/27     18:02                _posts
-a----       2019/12/27     18:02            750 .gitattributes
-a----       2019/12/27     18:02             80 .gitignore
-a----       2019/12/27     18:02            256 404.html
-a----       2019/12/27     18:02            451 aboutme.md
-a----       2019/12/27     18:02           2902 CHANGELOG.md
-a----       2019/12/27     18:02            159 Dockerfile
-a----       2019/12/27     18:02            966 feed.xml
-a----       2019/12/27     18:02            202 Gemfile
-a----       2019/12/27     18:46           6790 Gemfile.lock
-a----       2019/12/27     18:02           1963 index.html
-a----       2019/12/27     18:02           1281 LICENSE
-a----       2019/12/27     18:27          27461 README.md
-a----       2019/12/27     18:02           3937 staticman.yml
-a----       2019/12/27     18:02           1143 tags.html
-a----       2019/12/27     18:02           7098 _config.yml
```

```bash
$ bundle install
$ bundle exec jekyll serve
Configuration file: C:/Users/layca/workspace/BlogPosting/buddyrb/_config.yml
jekyll 3.7.4 | Error:  No source of timezone data could be found.
Please refer to http://tzinfo.github.io/datasourcenotfound for help resolving this error.
```
For the above error, install tzinfo-data and revise Gemfile
```bash
$ gem install tzinfo-data
```

The following description should be added into Gemfile (Windows10 case) :
```
$ gem 'tzinfo-data', platforms: [:mingw, :mswin, :x64_mingw]
```
Again, test jekyll serve
```bash
$ bundle exec jekyll serve
Configuration file: C:/Users/layca/workspace/BlogPosting/buddyrb/_config.yml
NOTE: Inheriting Faraday::Error::ClientError is deprecated; use Faraday::ClientError instead. It will be removed in or after version 1.0
Faraday::Error::ClientError.inherited called from C:/Ruby26-x64/lib/ruby/gems/2.6.0/gems/octokit-4.14.0/lib/octokit/middleware/follow_redirects.rb:14.
            Source: C:/Users/layca/workspace/BlogPosting/buddyrb
       Destination: C:/Users/layca/workspace/BlogPosting/buddyrb/_site
 Incremental build: disabled. Enable with --incremental
      Generating...
                    done in 5.675 seconds.
  Please add the following to your Gemfile to avoid polling for changes:
    gem 'wdm', '>= 0.1.0' if Gem.win_platform?
 Auto-regeneration: enabled for 'C:/Users/layca/workspace/BlogPosting/buddyrb'
    Server address: http://127.0.0.1:4000
  Server running... press ctrl-c to stop.
```

You can see your website at [http://127.0.0.1:4000](http://127.0.0.1:4000), 
and also when you revise your content, you can see the difference immediately on the local site. 

### 4. Add your own content

Add **.md** or **.html** file on `_post/` directory, then they will be posted automatically.
