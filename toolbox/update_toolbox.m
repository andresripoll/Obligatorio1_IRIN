function update_toolbox()
src0 = 'Z:\TDS\Jos� Parera\';
dst = 'E:\TDS-G33\toolbox';
locs = ['LS11'; 'LS12'; 'LS21'; 'LS22'; 'LS31'; 'LS32'; 'LS41'; 'LS42'];
folder = pwd;
if folder(end) == filesep
    folder = folder(1:end-1);
end
[~, fname, ~] = fileparts(folder);
idx = find(all(ismember(locs, fname, 'rows'), 2));
if idx > 1
	src = fullfile(src0, locs(idx-1,:), 'tboxupdt');
	files = dir(src);
	srcfs = files([files.isdir]==0);
	for k = 1:length(srcfs)
        fname = srcfs(k).name;
        dstf = fullfile(dst, fname);
        if ~isfile(dstf)
            disp(['Copiando ' fname]); 
            copyfile(fullfile(src, fname), dstf);
        else
            prompt = [sprintf('El fichero %s ya existe ', fname) ...
                      'en su caja de herramientas.\n' ...
                      '�Quiere sobrescribirlo? S/N [S]: '];
            reply = input(prompt, 's');
            if isempty(reply)
              reply = 'S';
            end
            if strcmpi(reply, 'S')
                disp(['Copiando ' fname]); 
                copyfile(fullfile(src, fname), dstf);
            end
        end
    end
end
end
