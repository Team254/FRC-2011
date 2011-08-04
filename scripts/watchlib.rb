/*
   Written by Parker Schuh
*/

module Watch
	$watchers = []
	def self.add(files,&block)
		$watchers.push(Watcher.new(files,block))
	end
	def self.main()
		while true
		$watchers.each do |watcher|
			watcher.check
			sleep(0.02)
		end
		end
	end
	class Watcher
		def initialize(files,block)
			@files = [files].flatten
			@block = block
			@last_time = get_mtime() - 10.0 #ensure a reload
		end
		def get_mtime()
			times = @files.collect do |file|
				if(File.exists?(file))
					File.stat(file).mtime
				else
					Time.now()
				end
			end
			times.max
		end
		def check()
			time = get_mtime()
			if(time - @last_time > 0.0)
				@block.call()
			end
			@last_time = time
		end
	end
end
Watch.add("../RobotConfig.csv") do
	system("sh copycsv.sh")
end
Watch.main
