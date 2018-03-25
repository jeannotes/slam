function kfs = kfstat(kfs, kf, flag)
% See also  kfupdate
global glv
    if isempty(kfs),  % kfs = kfstat([], kf)
        kfs.Ak0 = eye(kf.n);
        kfs.P0 = kf.Pxk;  kfs.Pk = kf.Pxk; kfs.Pk1 = kfs.Pk;
        for k=1:kf.n, kfs.Qjk{k} = zeros(kf.n); end
        for k=1:kf.m, kfs.Rsk{k} = zeros(kf.n); end
    elseif nargin>=3,  % kfs = kfstat(kfs, kf)
        if nargin==2, flag='B'; end
        if flag=='B'
            Bkk_1 = eye(kf.n)-kf.Kk*kf.Hk; Akk_1 = Bkk_1*kf.Phikk_1;
            kfs.Ak0 = Akk_1*kfs.Ak0;
            kfs.Pk1 = kfs.Ak0*kfs.P0*kfs.Ak0';
            for j=1:kf.n
                kfs.Qjk{j} = Akk_1*kfs.Qjk{j}*Akk_1' + kf.Qk(j,j)*Bkk_1(:,j)*Bkk_1(:,j)';
                kfs.Pk1 = kfs.Pk1 + kfs.Qjk{j};
            end
            for s=1:kf.m
                kfs.Rsk{s} = Akk_1*kfs.Rsk{s}*Akk_1' + kf.Rk(s,s)*kf.Kk(:,s)*kf.Kk(:,s)';
                kfs.Pk1 = kfs.Pk1 + kfs.Rsk{s};
            end
            kfs.Pk = Bkk_1*(kf.Phikk_1*kfs.Pk*kf.Phikk_1'+kf.Qk)*Bkk_1'+kf.Kk*kf.Rk*kf.Kk';
        elseif flag=='T'
            Bkk_1 = eye(kf.n); Akk_1 = Bkk_1*kf.Phikk_1;
            kfs.Ak0 = Akk_1*kfs.Ak0;
            kfs.Pk1 = kfs.Ak0*kfs.P0*kfs.Ak0';
            for j=1:kf.n
                kfs.Qjk{j} = Akk_1*kfs.Qjk{j}*Akk_1' + kf.Qk(j,j)*Bkk_1(:,j)*Bkk_1(:,j)';
                kfs.Pk1 = kfs.Pk1 + kfs.Qjk{j};
            end
            for s=1:kf.m
                kfs.Rsk{s} = Akk_1*kfs.Rsk{s}*Akk_1';
                kfs.Pk1 = kfs.Pk1 + kfs.Rsk{s};
            end
            kfs.Pk = kf.Phikk_1*kfs.Pk*kf.Phikk_1'+kf.Qk;
%             b=[kfs.Pk-kfs.Pk1]; max(max(abs(b)))
        elseif flag=='M'
            Bkk_1 = eye(kf.n)-kf.Kk*kf.Hk; Akk_1 = Bkk_1*eye(kf.n);
            kfs.Ak0 = Akk_1*kfs.Ak0;
            kfs.Pk1 = kfs.Ak0*kfs.P0*kfs.Ak0';
            for j=1:kf.n
                kfs.Qjk{j} = Akk_1*kfs.Qjk{j}*Akk_1';
                kfs.Pk1 = kfs.Pk1 + kfs.Qjk{j};
            end
            for s=1:kf.m
                kfs.Rsk{s} = Akk_1*kfs.Rsk{s}*Akk_1' + kf.Rk(s,s)*kf.Kk(:,s)*kf.Kk(:,s)';
                kfs.Pk1 = kfs.Pk1 + kfs.Rsk{s};
            end
            kfs.Pk = Bkk_1*kfs.Pk*Bkk_1'+kf.Kk*kf.Rk*kf.Kk';
        end         
    elseif nargin==2 && ~isempty(kfs)   %  kfs = kfstat(kfs, kf, 0)
        n = length(kfs.P0); m = length(kfs.Rsk);
        p = zeros(n); q = zeros(n); r = zeros(n,m);
        kfs.Pk0 = kfs.Ak0*kfs.P0*kfs.Ak0';
        for ll=1:n
            Pkll = 1;%kfs.Pk(ll,ll)+eps;
            for jj=1:n
                p(ll,jj) = kfs.Ak0(ll,jj)*kfs.P0(jj,jj)*kfs.Ak0(ll,jj)/Pkll;
                q(ll,jj) = kfs.Qjk{jj}(ll,ll)/Pkll;
            end
            for ss=1:m
                r(ll,ss) = kfs.Rsk{ss}(ll,ll)/Pkll;
            end
        end
        kfs.pqr = sqrt([p, q, r]);
%         for ll=1:2*n+m, kfs.pqr(:,ll) = kfs.pqr(:,ll)/max(abs(kfs.pqr(:,ll))); end
%         for ll=1:n, kfs.pqr(ll,:) = kfs.pqr(ll,:)/max(abs(kfs.pqr(ll,:))); end
        figure,% mesh(repmat((1:n)',1,2*n+m),repmat(1:2*n+m,n,1),kfs.pqr);
        subplot(321), bar(kfs.pqr(1:3,:)'/glv.min); grid on
        subplot(322), bar(kfs.pqr(4:6,:)'); grid on
        subplot(323), bar([kfs.pqr(7:8,:)'*glv.Re,kfs.pqr(9,:)']); grid on
        subplot(324), bar(kfs.pqr(10:12,:)'/glv.dph); grid on
        subplot(325), bar(kfs.pqr(13:15,:)'/glv.ug); grid on
        subplot(326), bar(kfs.pqr(16:18,:)'); grid on
        
    end
        